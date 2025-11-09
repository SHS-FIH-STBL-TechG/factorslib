// src/utils/utils.cpp
#include "utils/utils.h"
#include <algorithm>

/**
 * @file utils.cpp
 * @brief NmsBucketAggregator 的实现。核心思想：
 *   - 将任何到达的事件按（ts_ms / bucket_ms）对齐到一个桶；
 *   - 只要新来的时间跨越了当前桶的结束，就“产出旧桶，开启新桶”；
 *   - 行情用于计算 amount/volume 的增量；成交/委托仅作为桶内切片收集。
 */

namespace factorlib {

/** 确保当前时间桶已建立；若跨越，则把当前桶的起止更新时间到新桶 */
    bool NmsBucketAggregator::ensure_bucket(int64_t ts_ms, BucketOutputs& out) {
        int64_t bucket_start = (ts_ms / _bucket_ms) * _bucket_ms;
        int64_t bucket_end = bucket_start + _bucket_ms;

        if (!_has_bucket) {
            start_new_bucket(bucket_start);
            return false;
        }

        // 如果当前时间已经跨越了当前桶的边界
        if (ts_ms >= _cur.bucket_end_ms) {
            out = _cur;  // 产出当前桶
            // 重要：新桶的开始应该是当前桶的结束，而不是重新计算
            start_new_bucket(_cur.bucket_end_ms);
            return true;
        }

        return false;
    }

/** 开启一个以 new_start 为起点的新桶（不产出旧值，由调用者外部负责） */
void NmsBucketAggregator::start_new_bucket(int64_t new_start){
    _cur = BucketOutputs{};
    _cur.bucket_start_ms = new_start;
    _cur.bucket_end_ms = new_start + _bucket_ms;
    _has_bucket = true;
}

/** 喂入一条行情，用于：
 *  1) 计算成交额 amount/成交量 volume 的增量；
 *  2) 更新中价 midprice_last（取最新一笔）。
 */
    void NmsBucketAggregator::on_quote(const QuoteDepth& q) {
        if (!in_trading_session_ms(q.data_time_ms)) return;

        BucketOutputs temp_out;
        bool should_publish = ensure_bucket(q.data_time_ms, temp_out);
        // 注意：这里我们不能直接发布，因为聚合器不知道外部如何发布

        if (_has_last_quote && _last_trading_day == q.trading_day) {
            double d_amt = q.turnover - _last_turnover;
            int64_t d_vol = (q.volume >= _last_volume) ? (q.volume - _last_volume) : 0;
            _cur.amount_sum += d_amt;
            _cur.volume_sum += d_vol;
        } else {
            _cur.amount_sum += q.turnover;
            _cur.volume_sum += (int64_t)q.volume;
        }

        double mid = (q.bid_price > 0 && q.ask_price > 0) ?
                     (q.bid_price + q.ask_price) / 2.0 : _cur.midprice_last;
        _cur.midprice_last = mid;

        _has_last_quote = true;
        _last_turnover = q.turnover;
        _last_volume = q.volume;
        _last_trading_day = q.trading_day;
    }

/** 喂入成交：只做切片收集（不影响 amount/volume） */
    void NmsBucketAggregator::on_transaction(const Transaction& t) {
        if (!in_trading_session_ms(t.data_time_ms)) return;

        BucketOutputs temp_out;
        ensure_bucket(t.data_time_ms, temp_out); // 检查桶边界，但忽略返回值

        _cur.trans.push_back(t);
    }

/** 喂入委托：只做切片收集 */
    void NmsBucketAggregator::on_entrust(const Entrust& e) {
        if (!in_trading_session_ms(e.data_time_ms)) return;

        BucketOutputs temp_out;
        ensure_bucket(e.data_time_ms, temp_out); // 检查桶边界，但忽略返回值

        _cur.orders.push_back(e);
    }

/** 若 now_ms 已经跨过当前桶尾，则产出该桶并立即开启新桶 */
bool NmsBucketAggregator::flush_if_crossed(int64_t now_ms, BucketOutputs& out){
    if(!_has_bucket) return false;
    if(now_ms >= _cur.bucket_end_ms){
        out = _cur;
        start_new_bucket(_cur.bucket_end_ms);
        return true;
    }
    return false;
}

/** 强制产出当前桶（不检查是否跨桶） */
bool NmsBucketAggregator::force_flush(BucketOutputs& out){
    if(!_has_bucket) return false;
    out = _cur;
    _has_bucket = false;
    return true;
}

} // namespace factorlib
