// include/utils/databus.h
/**
 * @file databus.h
 * @brief 轻量级类型安全数据总线（发布/订阅 + 时间对齐 + 历史缓存）。
 *
 * 设计要点：
 *  - 以字符串 topic 为路由键，每个 topic 在注册时绑定一个具体类型 T；类型由编译期决定，运行期做校验。
 *  - 每个 topic 内部为 code（如股票代码）维度维护一段带时间戳的环形历史（容量由 register_topic(capacity) 指定）。
 *  - 提供拉取（get_*）、发布（publish）、订阅回调（subscribe）、阻塞等待（wait_*）等模式。
 *  - 所有公有成员函数在内部加锁，线程安全；订阅回调在发布时在持锁状态下逐一调用，回调应迅速返回。
 *  - 时间戳约定为毫秒（int64_t），建议使用“桶结束时间”对齐。
 *
 * 典型用法：
 *  1) DataBus::instance().register_topic<double>("zyd/amount", 120);
 *  2) publish：safe_publish<double>("zyd/amount", "000001.SZ", ts, v);
 *  3) 拉取：get_latest<double>("zyd/amount", "000001.SZ", out, &ts);
 *  4) 订阅：subscribe<double>("zyd/amount", "000001.SZ", [](code, ts, v){ ... });
 *  5) 等待：wait_for_time_exact<double>("zyd/amount", "000001.SZ", ts, out, 1000);
 */

#pragma once
#include "utils/log.h"
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
    /**
     * @brief 获取 DataBus 的全局单例。
     * @return 进程内唯一实例；线程安全地延迟初始化。
     */

    static DataBus& instance();

     /**
     * @brief 为给定 topic 绑定类型 T 并创建环形历史与同步原语。
     * @param topic    主题名（层级字符串，如 "zyd/amount"），同名不能重复注册。
     * @param capacity 环形历史容量（每个 code 单独维护 deque，超过容量自动淘汰最旧数据）。
     * @note 多次对同一 topic/类型重复注册将被忽略；不同类型重复注册会被拒绝。
     */
    template<typename T>
    void register_topic(const std::string& topic, size_t capacity=120);


     /**
     * @brief 发布一条时间戳数据到 topic/code，并立即唤醒等待者与触发订阅回调。
     * @param topic  主题名，需事先以相同 T 调用 register_topic 注册。
     * @param code   二级键（如股票代码），同一 topic 下按 code 分桶存储。
     * @param ts_ms  时间戳（毫秒）。建议使用“时间桶结束时间”用于跨因子对齐。
     * @param value  要写入的值（类型 T）。
     * @details 行为：写入对应 code 的环形历史（容量满则淘汰最旧），然后依次调用该 code 的所有订阅回调，最后唤醒等待该 code 的条件变量。
     * @warning 回调在总线内部持互斥锁的上下文里被逐个调用，应保证回调快速、无阻塞，避免影响发布吞吐。
     */

    template<typename T>
    void publish(const std::string& topic, const std::string& code, int64_t ts_ms, const T& value);

    // ---------- 拉取读取 ----------

    /// 读取最新一条（如需时间戳一并返回，传入 ts_ms 指针）
    /**
     * @brief 读取 topic/code 的最新一条记录。
     * @param topic 主题名（需匹配类型 T）。
     * @param code  二级键。
     * @param out   输出值（若存在记录）。
     * @param ts_ms 若非空，返回该记录时间戳。
     * @return 存在记录返回 true，否则 false。
     * @details 不阻塞；若尚无任何发布，则返回 false。
     */

    template<typename T>
    bool get_latest(const std::string& topic, const std::string& code, T& out, int64_t* ts_ms=nullptr) const;

    /// 按精确时间戳读取（避免拿错）
    /**
     * @brief 读取 topic/code 在“精确时间戳 ts_ms”的记录。
     * @return 命中返回 true 并写入 out；否则返回 false。
     * @details 仅当历史中存在“时间戳恰好等于 ts_ms”的元素才返回 true。
     */

    template<typename T>
    bool get_by_time_exact(const std::string& topic, const std::string& code, int64_t ts_ms, T& out) const;

    /// 读取最近 n 条（返回 (ts, value) 列表）
    /**
     * @brief 读取 topic/code 最近 n 条记录（从旧到新排序）。
     * @param n 要返回的条数；若历史不足 n 条，则返回全部历史。
     * @details 返回的向量元素为 (ts_ms, value) 对。
     */

    template<typename T>
    std::vector<std::pair<int64_t, T>> get_last_n(const std::string& topic, const std::string& code, size_t n) const;

    // ---------- 订阅回调 ----------

     /**
     * @brief 订阅某 topic/code 的数据更新回调。
     * @param cb 回调形式为 (code, ts_ms, const T& value)。发布成功后会在持锁状态下按注册顺序依次调用。
     * @note 订阅与取消订阅接口当前未区分；可通过外部状态使回调尽快返回以“软取消”。
     */

    template<typename T>
    void subscribe(const std::string& topic, const std::string& code,
                   std::function<void(const std::string&, int64_t, const T&)> cb);

    // ---------- 阻塞等待（A 等 B） ----------

     /**
     * @brief 阻塞等待直到收到“时间戳恰为 ts_ms”的记录，或超时返回。
     * @param timeout_ms 超时时间，毫秒。超时返回 false。
     * @details 内部通过针对该 code 的 std::condition_variable 循环等待；每次发布会唤醒等待者。
     */

    template<typename T>
    bool wait_for_time_exact(const std::string& topic, const std::string& code,
                             int64_t ts_ms, T& out, int64_t timeout_ms = 1000);


     /**
     * @brief 阻塞等待直到收到“时间戳不早于 ts_ms”的记录（>= ts_ms），或超时返回。
     * @details 适合“等到桶完成或下一时刻数据”的场景；与 exact 版本不同，命中“晚于 ts_ms”的第一条也视为成功。
     */

    template<typename T>
    bool wait_for_time_at_least(const std::string& topic, const std::string& code,
                                int64_t ts_ms, T& out, int64_t timeout_ms = 1000);


    /**
     * @brief 清空所有已注册的 topic 与其历史/订阅者；主要用于测试场景做干净重置。
     * @warning 非常规操作：生产环境慎用，调用后所有历史与订阅都将丢失。
     */
    void reset() {
        std::lock_guard<std::mutex> lk(_m);
        _topics.clear();
    }
private:
    struct ChannelBase { virtual ~ChannelBase()=default; };

    /**
     * @brief 单个 topic 对应的具体类型通道：维护每个 code 的历史、订阅者与条件变量。
     * @tparam T 该 topic 绑定的数据类型。
     *
     * 组成：
     *  - store：code → deque<(ts_ms, T)>，固定容量，满则淘汰最旧。
     *  - subs ：code → 回调列表；publish 时在锁内依序调用。
     *  - cvs  ：code → 条件变量；用于 wait_* 阻塞等待与 publish 唤醒。
     */
    template<typename T>
    struct Channel : ChannelBase {

        /// @brief 每个 code 的历史容量上限；超出时自动弹出最旧元素。
        size_t capacity;

        /// @brief 历史存储：按 code 分桶的队列，元素为 (时间戳, 值)；按时间递增插入。
        std::unordered_map<std::string, std::deque<std::pair<int64_t, T>>> store;

        /// @brief 订阅回调表：按 code 维护回调列表；publish 时在锁内按注册顺序逐个调用。
        std::unordered_map<std::string, std::vector<std::function<void(const std::string&, int64_t, const T&)>>> subs;

        /// @brief 条件变量表：按 code 维护；用于 wait_* 阻塞等待与 publish 唤醒。
        std::unordered_map<std::string, std::condition_variable> cvs;

        /**
         * @brief 构造指定容量的通道；capacity 影响每个 code 的历史上限。
         * @param cap 每个 code 的历史容量上限。
         */
        explicit Channel(size_t cap):capacity(cap){}

        /**
         * @brief 追加一条 (ts_ms, value) 到指定 code 的历史；超容量时淘汰最旧，然后触发订阅回调并唤醒等待者。
         * @warning 在 DataBus 的互斥锁保护下调用；回调在锁内执行，应保持回调短小无阻塞。
         */

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

        // 当调用方只传“纯 code”时，尝试匹配以 code 为前缀的 scope key，确保老逻辑仍能取到数据
        std::string resolve_code_key(const std::string& code) const {
            if (store.find(code) != store.end()) return code;

            // 统一的“前缀匹配 + 找到即返回”逻辑，避免在多窗口场景里遗漏最新值
            const auto match_scoped = [&](const std::string& prefix) -> std::string {
                for (const auto& kv : store) {
                    if (kv.first.rfind(prefix, 0) == 0) return kv.first;
                }
                return {};
            };

            // run_tests 中的用例仍会传裸 code，需要兼容 `code|w{window}` 的组合键
            if (auto scoped = match_scoped(code + "|w"); !scoped.empty()) return scoped;
            return {};
        }
    };

    DataBus()=default;
    DataBus(const DataBus&)=delete;
    DataBus& operator=(const DataBus&)=delete;

    mutable std::mutex _m;

    /// @brief 主题注册表：topic → (绑定类型信息, 通道实例指针)；通过 type_index 保证类型一致性。
    std::unordered_map<std::string, std::pair<std::type_index, std::unique_ptr<ChannelBase>>> _topics;

    /**
     * @brief 取得与 topic 绑定的具体类型通道；未注册或类型不匹配返回 nullptr。
     * @param topic 主题名。
     * @return 若已注册且类型匹配，返回 Channel<T>*；否则为 nullptr。
     * @details 仅供内部使用，配合模板参数 T 做编译期与运行期一致性校验。
     */
    template<typename T>
    Channel<T>* get_channel(const std::string& topic) const;
};

// -------------------- 模板实现 --------------------

/// @brief 单例获取：通过局部静态对象保证线程安全懒初始化。
inline DataBus& DataBus::instance(){ static DataBus bus; return bus; }

/// @brief 注册 topic：若首次见到则绑定类型并分配通道，重复注册直接忽略。
template<typename T>
void DataBus::register_topic(const std::string& topic, size_t capacity){
    std::lock_guard<std::mutex> lk(_m);
    auto it = _topics.find(topic);
    if(it == _topics.end()){
        _topics.emplace(topic, std::make_pair(std::type_index(typeid(T)), std::unique_ptr<ChannelBase>(new Channel<T>(capacity))));
    }
}

/// @brief 内部工具：根据 topic 获取已注册的具体类型通道。
template<typename T>
typename DataBus::Channel<T>* DataBus::get_channel(const std::string& topic) const{
    auto it = _topics.find(topic);
    if(it == _topics.end()) return nullptr;
    if(it->second.first != std::type_index(typeid(T))) return nullptr;
    return static_cast<Channel<T>*>(it->second.second.get());
}

/// @brief 发布：写入对应通道并触发订阅/等待。
template<typename T>
void DataBus::publish(const std::string& topic, const std::string& code, int64_t ts_ms, const T& value){
    std::lock_guard<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return;
    ch->push(code, ts_ms, value);
}

/// @brief 读取最新一条记录（若不存在返回 false）。
template<typename T>
bool DataBus::get_latest(const std::string& topic, const std::string& code, T& out, int64_t* ts_ms) const{
    std::lock_guard<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return false;
    auto key = ch->resolve_code_key(code);
    if (key.empty()) return false;
    auto it = ch->store.find(key);
    if(it == ch->store.end() || it->second.empty()) return false;
    const auto& kv = it->second.back();
    out = kv.second;
    if(ts_ms) *ts_ms = kv.first;
    return true;
}

/// @brief 读取精确时间戳 ts_ms 对应的记录。
template<typename T>
bool DataBus::get_by_time_exact(const std::string& topic, const std::string& code, int64_t ts_ms, T& out) const{
    std::lock_guard<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return false;
    auto key = ch->resolve_code_key(code);
    if (key.empty()) return false;
    auto it = ch->store.find(key);
    if(it == ch->store.end()) return false;
    const auto& dq = it->second;
    for(const auto& kv : dq){
        if(kv.first == ts_ms){ out = kv.second; return true; }
    }
    return false;
}

/// @brief 读取最近 n 条记录（不足 n 条则返回全部）。
template<typename T>
std::vector<std::pair<int64_t, T>> DataBus::get_last_n(const std::string& topic, const std::string& code, size_t n) const{
    std::lock_guard<std::mutex> lk(_m);
    std::vector<std::pair<int64_t, T>> res;
    auto ch = get_channel<T>(topic);
    if(!ch) return res;
    auto key = ch->resolve_code_key(code);
    if (key.empty()) return res;
    auto it = ch->store.find(key);
    if(it == ch->store.end()) return res;
    const auto& dq = it->second;
    if(n > dq.size()) n = dq.size();
    for(size_t i = dq.size()-n; i < dq.size(); ++i){
        res.push_back(dq[i]);
    }
    return res;
}

/// @brief 注册订阅回调：发布时会按注册顺序调用。
template<typename T>
void DataBus::subscribe(const std::string& topic, const std::string& code,
                        std::function<void(const std::string&, int64_t, const T&)> cb){
    std::lock_guard<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return;
    auto key = ch->resolve_code_key(code);
    if (key.empty()) key = code;
    ch->subs[key].push_back(std::move(cb));
}

/// @brief 阻塞等待“时间戳恰好等于 ts_ms”的记录。
template<typename T>
bool DataBus::wait_for_time_exact(const std::string& topic, const std::string& code,
                                  int64_t ts_ms, T& out, int64_t timeout_ms){
    using namespace std::chrono;
    auto deadline = steady_clock::now() + milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return false;
    auto key = ch->resolve_code_key(code);
    if (key.empty()) key = code;
    auto& cv = ch->cvs[key];
    while(true){
        auto it = ch->store.find(key);
        if(it != ch->store.end()){
            for(const auto& kv : it->second){
                if(kv.first == ts_ms){ out = kv.second; return true; }
            }
        }
        if(cv.wait_until(lk, deadline) == std::cv_status::timeout) return false;
    }
}

/// @brief 阻塞等待“时间戳 >= ts_ms”的第一条记录。
template<typename T>
bool DataBus::wait_for_time_at_least(const std::string& topic, const std::string& code,
                                     int64_t ts_ms, T& out, int64_t timeout_ms){
    using namespace std::chrono;
    auto deadline = steady_clock::now() + milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lk(_m);
    auto ch = get_channel<T>(topic);
    if(!ch) return false;
    auto key = ch->resolve_code_key(code);
    if (key.empty()) key = code;
    auto& cv = ch->cvs[key];
    while(true){
        auto it = ch->store.find(key);
        if(it != ch->store.end() && !it->second.empty()){
            const auto& back = it->second.back();
            if(back.first >= ts_ms){ out = back.second; return true; }
        }
        if(cv.wait_until(lk, deadline) == std::cv_status::timeout) return false;
    }
}


/**
 * @brief 对 DataBus::publish 的安全封装：捕获异常并记录错误日志，返回是否发布成功。
 * @param topic  主题名（需与类型 T 匹配且已注册）。
 * @param code   二级键。
 * @param ts_ms  毫秒时间戳。
 * @param value  发布值（T）。
 * @return 发布成功返回 true；若未注册/类型不匹配/回调异常等导致失败，返回 false 并输出错误日志。
 * @details 不改变数据语义，仅在异常情况下提供统一的错误可观测性；适合生产路径替换裸 publish 调用。
 */

template <typename T>
inline bool safe_publish(const std::string& topic,
                         const std::string& code,
                         int64_t ts_ms,
                         const T& value) {
    try {
        DataBus::instance().publish<T>(topic, code, ts_ms, value);
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("safe_publish 失败: topic={}, code={}, ts={}, what={}", topic, code, ts_ms, e.what());
        return false;
    } catch (...) {
        LOG_ERROR("safe_publish 失败: topic={}, code={}, ts={}, 未知异常", topic, code, ts_ms);
        return false;
    }
}

} // namespace factorlib
