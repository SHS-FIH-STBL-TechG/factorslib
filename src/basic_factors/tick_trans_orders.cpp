
#include "basic_factors/tick_trans_orders.h"

/**
 * @file tick_trans_orders.cpp
 * @brief 0109 的“翻译”版本实现：
 *  - 聚合到 N 毫秒桶；
 *  - 产出五类指标；
 *  - 发布到 DataBus（层级 topic）。
 */

namespace factorlib {

// 层级 topic 常量
static const char* TOP_AMOUNT = "zyd/amount";
static const char* TOP_VOLUME = "zyd/volume";
static const char* TOP_MID    = "zyd/midprice";
static const char* TOP_TTRANS = "zyd/tick/trans";
static const char* TOP_TORD   = "zyd/tick/orders";

/** 注册五个 topic；模板化保证类型安全 */
void TickTransOrders::register_topics(size_t cap){
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_AMOUNT, cap);
    bus.register_topic<int64_t>(TOP_VOLUME, cap);
    bus.register_topic<double>(TOP_MID, cap);
    bus.register_topic<std::vector<Transaction>>(TOP_TTRANS, cap);
    bus.register_topic<std::vector<Entrust>>(TOP_TORD, cap);
}

/** 确保每个代码都有自己的聚合器（惰性创建） */
void TickTransOrders::ensure_code(const std::string& code){
    if(_agg.find(code) == _agg.end()){
        _agg.emplace(code, NmsBucketAggregator(_cfg.bucket_size_ms));
    }
}

/** 若跨桶则先发布上一桶 */
void TickTransOrders::maybe_flush_and_publish(const std::string& code, int64_t now_ms){
    auto it = _agg.find(code);
    if(it == _agg.end()) return;
    BucketOutputs out;
    if(it->second.flush_if_crossed(now_ms, out)){
        publish_bucket(code, out);
    }
}

/** 发布一个桶到 DataBus，时间戳使用“桶结束时间” */
void TickTransOrders::publish_bucket(const std::string& code, const BucketOutputs& out){
    auto ts = out.bucket_end_ms; // 以桶结束时刻作为产出时间
    auto& bus = DataBus::instance();
    bus.publish<double>(TOP_AMOUNT, code, ts, out.amount_sum);
    bus.publish<int64_t>(TOP_VOLUME, code, ts, out.volume_sum);
    bus.publish<double>(TOP_MID, code, ts, out.midprice_last);
    bus.publish<std::vector<Transaction>>(TOP_TTRANS, code, ts, out.trans);
    bus.publish<std::vector<Entrust>>(TOP_TORD, code, ts, out.orders);
    TSET_INFO("publish %s [%lld,%lld) amt=%.2f vol=%lld",
        code.c_str(),
        (long long)out.bucket_start_ms, (long long)out.bucket_end_ms,
        out.amount_sum, (long long)out.volume_sum);
}

/** 行情到达：推进聚合，必要时产出上一桶 */
void TickTransOrders::on_quote(const QuoteDepth& q){
    if(q.instrument_id.empty()) return;
    if(!in_trading_session_ms(q.data_time_ms)) return;
    ensure_code(q.instrument_id);
    maybe_flush_and_publish(q.instrument_id, q.data_time_ms);
    _agg[q.instrument_id].on_quote(q);
}

/** 成交到达：推进聚合（只切片），必要时产出上一桶 */
void TickTransOrders::on_transaction(const Transaction& t){
    if(t.instrument_id.empty()) return;
    if(!in_trading_session_ms(t.data_time_ms)) return;
    ensure_code(t.instrument_id);
    maybe_flush_and_publish(t.instrument_id, t.data_time_ms);
    _agg[t.instrument_id].on_transaction(t);
}

/** 委托到达：推进聚合（只切片），必要时产出上一桶 */
void TickTransOrders::on_entrust(const Entrust& e){
    if(e.instrument_id.empty()) return;
    if(!in_trading_session_ms(e.data_time_ms)) return;
    ensure_code(e.instrument_id);
    maybe_flush_and_publish(e.instrument_id, e.data_time_ms);
    _agg[e.instrument_id].on_entrust(e);
}

/** 强制产出某代码的当前桶（用于测试或收盘） */
bool TickTransOrders::force_flush(const std::string& code){
    auto it = _agg.find(code);
    if(it == _agg.end()) return false;
    BucketOutputs out;
    if(it->second.force_flush(out)){
        publish_bucket(code, out);
        return true;
    }
    return false;
}

} // namespace factorlib
