// src/utils/nms_bucket_aggregator.cpp
#include "utils/nms_bucket_aggregator.h"
#include <algorithm>

namespace factorlib {

NmsBucketAggregator::NmsBucketAggregator(int64_t bucket_ms)
    : _bucket_ms(bucket_ms) {}

void NmsBucketAggregator::set_bucket_ms(int64_t ms) {
    _bucket_ms = ms;
}

bool NmsBucketAggregator::ensure_bucket(int64_t ts_ms, BucketOutputs& out) {
    int64_t bucket_start = (ts_ms / _bucket_ms) * _bucket_ms;
    int64_t bucket_end = bucket_start + _bucket_ms;

    if (!_has_bucket) {
        start_new_bucket(bucket_start);
        return false;
    }

    if (ts_ms >= _cur.bucket_end_ms) {
        out = _cur;
        start_new_bucket(_cur.bucket_end_ms);
        return true;
    }

    return false;
}

void NmsBucketAggregator::start_new_bucket(int64_t new_start) {
    _cur = BucketOutputs{};
    _cur.bucket_start_ms = new_start;
    _cur.bucket_end_ms = new_start + _bucket_ms;
    _has_bucket = true;
}

void NmsBucketAggregator::on_quote(const QuoteDepth& q) {
    if (!TradingTime::in_trading_session_ms(q.data_time_ms)) return;

    BucketOutputs temp_out;
    bool should_publish = ensure_bucket(q.data_time_ms, temp_out);

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

void NmsBucketAggregator::on_transaction(const Transaction& t) {
    if (!TradingTime::in_trading_session_ms(t.data_time_ms)) return;

    BucketOutputs temp_out;
    ensure_bucket(t.data_time_ms, temp_out);
    _cur.trans.push_back(t);
}

void NmsBucketAggregator::on_entrust(const Entrust& e) {
    if (!TradingTime::in_trading_session_ms(e.data_time_ms)) return;

    BucketOutputs temp_out;
    ensure_bucket(e.data_time_ms, temp_out);
    _cur.orders.push_back(e);
}

bool NmsBucketAggregator::flush_if_crossed(int64_t now_ms, BucketOutputs& out) {
    if (!_has_bucket) return false;
    if (now_ms >= _cur.bucket_end_ms) {
        out = _cur;
        start_new_bucket(_cur.bucket_end_ms);
        return true;
    }
    return false;
}

bool NmsBucketAggregator::force_flush(BucketOutputs& out) {
    if (!_has_bucket) return false;
    out = _cur;
    _has_bucket = false;
    return true;
}

} // namespace factorlib