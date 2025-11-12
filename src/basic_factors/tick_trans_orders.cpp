// src/basic_factors/tick_trans_orders.cpp
#include "basic_factors/tick_trans_orders.h"
#include "utils/log.h"
#include "utils/databus.h"  // for safe_publish

/**
 * @file tick_trans_orders.cpp
 * @brief 0109 的“翻译”版本实现：
 *  - 聚合到 N 毫秒桶；
 *  - 产出五类指标（amount/volume/midprice/tick.trans/tick.orders）；
 *  - 可选产出两个 tick 之间的切片（interval/trans, interval/orders）；
 *  - 发布到 DataBus（层级 topic）。
 */

namespace factorlib {

    // 层级 topic 常量
    static const char* TOP_AMOUNT  = "zyd/amount";
    static const char* TOP_VOLUME  = "zyd/volume";
    static const char* TOP_MID     = "zyd/midprice";
    static const char* TOP_TTRANS  = "zyd/tick/trans";
    static const char* TOP_TORD    = "zyd/tick/orders";
    static const char* TOP_IVTRANS = "zyd/interval/trans";
    static const char* TOP_IVORD   = "zyd/interval/orders";

    // ---- TickTransOrders ----

    void TickTransOrders::register_topics(size_t cap){
        auto& bus = DataBus::instance();
        // 时间桶五个主题
        bus.register_topic<double>(TOP_AMOUNT, cap);
        bus.register_topic<int64_t>(TOP_VOLUME, cap);
        bus.register_topic<double>(TOP_MID, cap);
        bus.register_topic<std::vector<Transaction>>(TOP_TTRANS, cap);
        bus.register_topic<std::vector<Entrust>>(TOP_TORD, cap);
        // interval 两个主题（静态函数无法读实例配置，这里总是注册，发布开关由实例控制）
        bus.register_topic<std::vector<Transaction>>(TOP_IVTRANS, cap);
        bus.register_topic<std::vector<Entrust>>(TOP_IVORD, cap);
    }

    void TickTransOrders::ensure_code(const std::string& code){
        if (_agg.find(code) == _agg.end()){
            _agg.emplace(code, NmsBucketAggregator(_cfg.bucket_size_ms));
            _last_tick_ms[code] = 0;
            _interval_trans_pending[code] = {};
            _interval_orders_pending[code] = {};
        }
        _agg[code].set_bucket_ms(_cfg.bucket_size_ms);
    }

    void TickTransOrders::publish_bucket(const std::string& code, const BucketOutputs& out){
        auto& bus = DataBus::instance();
        // 使用桶的结束时间作为时间戳
        const int64_t ts = out.bucket_end_ms;
        safe_publish<double>(TOP_AMOUNT, code, ts, out.amount_sum);
        safe_publish<int64_t>(TOP_VOLUME, code, ts, out.volume_sum);
        safe_publish<double>(TOP_MID, code, ts, out.midprice_last);
        safe_publish<std::vector<Transaction>>(TOP_TTRANS, code, ts, out.trans);
        safe_publish<std::vector<Entrust>>(TOP_TORD, code, ts, out.orders);
    }

    void TickTransOrders::maybe_flush_and_publish(const std::string& code, int64_t now_ms) {
        BucketOutputs out;

    if (_agg[code].ensure_bucket(now_ms, out)) {
        publish_bucket(code, out);
        return;
    }

    if (_agg[code].flush_if_crossed(now_ms, out)) {
        publish_bucket(code, out);
    }
}

    void TickTransOrders::on_quote(const QuoteDepth& q) {
    ensure_code(q.instrument_id);

    // interval逻辑保持不变
    if (_cfg.emit_tick_interval) {
        auto &lt = _last_tick_ms[q.instrument_id];
        if (lt > 0) {
            auto &vt = _interval_trans_pending[q.instrument_id];
            auto &vo = _interval_orders_pending[q.instrument_id];
            safe_publish<std::vector<Transaction>>(TOP_IVTRANS, q.instrument_id, q.data_time_ms, vt);
            safe_publish<std::vector<Entrust>>(TOP_IVORD, q.instrument_id, q.data_time_ms, vo);
            vt.clear();
            vo.clear();
        }
        _last_tick_ms[q.instrument_id] = q.data_time_ms;
    }

    // 在调用聚合器之前先检查桶边界
    maybe_flush_and_publish(q.instrument_id, q.data_time_ms);

    // 处理当前行情
    _agg[q.instrument_id].on_quote(q);

    // 再次检查，确保没有遗漏
    maybe_flush_and_publish(q.instrument_id, q.data_time_ms);
}

    void TickTransOrders::on_tick(const CombinedTick& x){
        if (x.kind == CombinedKind::Trade) {
            Transaction t{};
            t.instrument_id = x.instrument_id;
            t.data_time_ms  = x.data_time_ms;
            t.main_seq      = x.main_seq;
            t.price         = x.price;
            t.side          = x.side;
            t.volume        = x.volume;
            t.bid_no        = x.bid_no;
            t.ask_no        = x.ask_no;

            ensure_code(t.instrument_id);
            if (_cfg.emit_tick_interval){
                auto lt = _last_tick_ms[t.instrument_id];
                if (lt > 0 && t.data_time_ms > lt){
                    _interval_trans_pending[t.instrument_id].push_back(t);
                }
            }
            _agg[t.instrument_id].on_transaction(t);
        } else {
            Entrust e{};
            e.instrument_id = x.instrument_id;
            e.data_time_ms  = x.data_time_ms;
            e.main_seq      = x.main_seq;
            e.price         = x.price;
            e.side          = x.side;
            e.volume        = x.volume;
            e.order_id      = x.order_id;

            ensure_code(e.instrument_id);
            if (_cfg.emit_tick_interval){
                auto lt = _last_tick_ms[e.instrument_id];
                if (lt > 0 && e.data_time_ms > lt){
                    _interval_orders_pending[e.instrument_id].push_back(e);
                }
            }
            _agg[e.instrument_id].on_entrust(e);
        }
    }


bool TickTransOrders::force_flush(const std::string& code){
    auto it = _agg.find(code);
    if (it == _agg.end()) return false;
    BucketOutputs out;
    if (it->second.force_flush(out)){
        publish_bucket(code, out);
        return true;
    }
    return false;
}

} // namespace factorlib