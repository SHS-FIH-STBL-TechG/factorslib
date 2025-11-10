// include/utils/databus.h
#pragma once
#include <string>
#include <unordered_map>
#include <deque>
#include <vector>
#include <memory>
#include <mutex>
#include <typeindex>
#include <typeinfo>
#include <condition_variable>
#include <chrono>
#include <functional>
#include <iostream>
#include <ostream>
#include <shared_mutex>

#include "log.h"
#include "utils/trading_time.h"

/**
 * @file databus.h
 * @brief 模板化数据总线：
 *  - 每个 topic 绑定一个类型 T（register_topic<T>）；
 *  - 支持按 (topic, code, ts_ms) 维度的发布/读取/等待；
 *  - 保留环形历史（仅按数量淘汰）；
 *  - 提供订阅回调与阻塞等待（1s 超时默认）。
 * 设计目的：实现“基础因子产出→被复杂因子安全复用（避免读错类型/时间）”。
 */

namespace factorlib {

class DataBus {
public:
    static DataBus& instance();

    // ---------- 注册与发布 ----------

    /**
     * @brief 注册一个主题，并绑定其类型为 T；需要指定环形容量（历史条数）
     * @param topic   层级主题名（如 "zyd/amount"）
     * @param capacity 环形容量（如 120）
     */
    template<typename T>
    void register_topic(const std::string& topic, size_t capacity=120);

    /**
     * @brief 发布一条数据（写入环形历史并唤醒订阅者/等待者）
     * @param topic  主题名（需与注册时类型一致）
     * @param code   股票/合约代码（如 "000001.SZ"）
     * @param ts_ms  时间戳（建议用“桶结束时间”）
     * @param value  数据
     */
    template<typename T>
    void publish(const std::string& topic, const std::string& code, int64_t ts_ms, const T& value);

    // ---------- 拉取读取 ----------

    /// 读取最新一条（如需时间戳一并返回，传入 ts_ms 指针）
    template<typename T>
    bool get_latest(const std::string& topic, const std::string& code, T& out, int64_t* ts_ms=nullptr) const;

    /// 按精确时间戳读取（避免拿错）
    template<typename T>
    bool get_by_time_exact(const std::string& topic, const std::string& code, int64_t ts_ms, T& out) const;

    /// 读取最近 n 条（返回 (ts, value) 列表）
    template<typename T>
    std::vector<std::pair<int64_t, T>> get_last_n(const std::string& topic, const std::string& code, size_t n) const;

    // ---------- 订阅回调 ----------

    /**
     * @brief 订阅某 (topic, code) 的新数据（发布时会回调一次）
     * @param cb 回调函数形如 (code, ts_ms, value)
     */
    template<typename T>
    void subscribe(const std::string& topic, const std::string& code,
                   std::function<void(const std::string&, int64_t, const T&)> cb);

    // ---------- 阻塞等待（A 等 B） ----------

    /**
     * @brief 等待到“精确时间戳”的数据（默认超时 1000ms，返回 false）
     */
    template<typename T>
    bool wait_for_time_exact(const std::string& topic, const std::string& code,
                             int64_t ts_ms, T& out, int64_t timeout_ms = 1000);

    /**
     * @brief 等待到“时间戳不早于 ts_ms”的数据（默认超时 1000ms）
     */
    template<typename T>
    bool wait_for_time_at_least(const std::string& topic, const std::string& code,
                                int64_t ts_ms, T& out, int64_t timeout_ms = 1000);


    void reset() {
        std::lock_guard<std::mutex> lk(_m);
        _topics.clear();
    }
private:
    struct ChannelBase { virtual ~ChannelBase()=default; };

    /**
     * @brief 单 topic 的通道，绑定具体类型 T
     *  - store: code → deque<(ts, value)>
     *  - subs : code → 回调列表
     *  - cvs  : code → 条件变量，用于等待/唤醒
     */
    template<typename T>
    struct Channel : ChannelBase {
        size_t capacity;
        std::unordered_map<std::string, std::deque<std::pair<int64_t, T>>> store;
        std::unordered_map<std::string, std::vector<std::function<void(const std::string&, int64_t, const T&)>>> subs;
        std::unordered_map<std::string, std::condition_variable> cvs;
        explicit Channel(size_t cap):capacity(cap){}
        void push(const std::string& code, int64_t ts, const T& v){
            auto& dq = store[code];
            dq.emplace_back(ts, v);
            if(dq.size() > capacity) dq.pop_front();
            cvs[code].notify_all();
            auto it = subs.find(code);
            if(it!=subs.end()){
                for(auto& f: it->second){ f(code, ts, v); }
            }
        }
    };

    DataBus()=default;
    DataBus(const DataBus&)=delete;
    DataBus& operator=(const DataBus&)=delete;

    mutable std::mutex _m;
    // topic → (类型信息, Channel 对象)
    std::unordered_map<std::string, std::pair<std::type_index, std::unique_ptr<ChannelBase>>> _topics;

    template<typename T>
    Channel<T>* get_channel(const std::string& topic) const;
};

// -------------------- 模板实现 --------------------

inline DataBus& DataBus::instance(){ static DataBus bus; return bus; }

template<typename T>
void DataBus::register_topic(const std::string& topic, size_t capacity){
    std::lock_guard<std::mutex> lk(_m);
    auto it = _topics.find(topic);
    if(it == _topics.end()){
        _topics.emplace(topic, std::make_pair(std::type_index(typeid(T)), std::unique_ptr<ChannelBase>(new Channel<T>(capacity))));
    }
}

template<typename T>
typename DataBus::Channel<T>* DataBus::get_channel(const std::string& topic) const{
    auto it = _topics.find(topic);
    if(it == _topics.end()) return nullptr;
    if(it->second.first != std::type_index(typeid(T))) return nullptr;
    return static_cast<Channel<T>*>(it->second.second.get());
}

template<typename T>
void DataBus::publish(const std::string& topic, const std::string& code, int64_t ts_ms, const T& value){
    std::lock_guard<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return;
    ch->push(code, ts_ms, value);
}

template<typename T>
bool DataBus::get_latest(const std::string& topic, const std::string& code, T& out, int64_t* ts_ms) const{
    std::lock_guard<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return false;
    auto it = ch->store.find(code);
    if(it == ch->store.end() || it->second.empty()) return false;
    const auto& kv = it->second.back();
    out = kv.second;
    if(ts_ms) *ts_ms = kv.first;
    return true;
}

template<typename T>
bool DataBus::get_by_time_exact(const std::string& topic, const std::string& code, int64_t ts_ms, T& out) const{
    std::lock_guard<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return false;
    auto it = ch->store.find(code);
    if(it == ch->store.end()) return false;
    const auto& dq = it->second;
    for(const auto& kv : dq){
        if(kv.first == ts_ms){ out = kv.second; return true; }
    }
    return false;
}

template<typename T>
std::vector<std::pair<int64_t, T>> DataBus::get_last_n(const std::string& topic, const std::string& code, size_t n) const{
    std::lock_guard<std::mutex> lk(_m);
    std::vector<std::pair<int64_t, T>> res;
    auto ch = get_channel<T>(topic);
    if(!ch) return res;
    auto it = ch->store.find(code);
    if(it == ch->store.end()) return res;
    const auto& dq = it->second;
    if(n > dq.size()) n = dq.size();
    for(size_t i = dq.size()-n; i < dq.size(); ++i){
        res.push_back(dq[i]);
    }
    return res;
}

template<typename T>
void DataBus::subscribe(const std::string& topic, const std::string& code,
                        std::function<void(const std::string&, int64_t, const T&)> cb){
    std::lock_guard<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return;
    ch->subs[code].push_back(std::move(cb));
}

template<typename T>
bool DataBus::wait_for_time_exact(const std::string& topic, const std::string& code,
                                  int64_t ts_ms, T& out, int64_t timeout_ms){
    using namespace std::chrono;
    auto deadline = steady_clock::now() + milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return false;
    auto& cv = ch->cvs[code];
    while(true){
        auto it = ch->store.find(code);
        if(it != ch->store.end()){
            for(const auto& kv : it->second){
                if(kv.first == ts_ms){ out = kv.second; return true; }
            }
        }
        if(cv.wait_until(lk, deadline) == std::cv_status::timeout) return false;
    }
}

template<typename T>
bool DataBus::wait_for_time_at_least(const std::string& topic, const std::string& code,
                                     int64_t ts_ms, T& out, int64_t timeout_ms){
    using namespace std::chrono;
    auto deadline = steady_clock::now() + milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return false;
    auto& cv = ch->cvs[code];
    while(true){
        auto it = ch->store.find(code);
        if(it != ch->store.end() && !it->second.empty()){
            const auto& back = it->second.back();
            if(back.first >= ts_ms){ out = back.second; return true; }
        }
        if(cv.wait_until(lk, deadline) == std::cv_status::timeout) return false;
    }
}

} // namespace factorlib
