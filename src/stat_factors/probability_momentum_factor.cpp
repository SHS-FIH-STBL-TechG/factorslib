#include "probability_momentum_factor.h"

#include <cmath>
#include <limits>

#include "utils/databus.h"
#include "utils/log.h"
#include "math/numeric_utils.h"
#include "math/distributions.h"
#include "../config/runtime_config.h"

using factorlib::config::RC;

namespace factorlib {

using factorlib::math::NumericUtils;
using factorlib::math::Distributions;

// =====================[ 构造 & 配置 ]=====================

ProbabilityMomentumFactor::ProbabilityMomentumFactor(
        const std::vector<std::string>& codes,
        const ProbMomentumConfig& cfg)
    : BaseFactor("ProbMomentumFactor", codes)
    , _cfg(cfg) {

    // 从运行时配置读取参数（带默认值）
    int ws = RC().geti("prob_mom.window_size", _cfg.window_size);
    if (ws <= 1) {
        LOG_WARN("ProbabilityMomentumFactor: window_size={} 无效，重置为 60", ws);
        ws = 60;
    }
    _cfg.window_size = ws;

    double min_w = RC().getd("prob_mom.min_total_weight", _cfg.min_total_weight);
    if (!(min_w > 0.0)) {
        LOG_WARN("ProbabilityMomentumFactor: min_total_weight={} 无效，重置为 1.0", min_w);
        min_w = 1.0;
    }
    _cfg.min_total_weight = min_w;

    double min_sigma = RC().getd("prob_mom.min_sigma", _cfg.min_sigma);
    if (!(min_sigma > 0.0)) {
        LOG_WARN("ProbabilityMomentumFactor: min_sigma={} 无效，重置为 1e-6", min_sigma);
        min_sigma = 1e-6;
    }
    _cfg.min_sigma = min_sigma;

    double buy_thr  = RC().getd("prob_mom.buy_threshold",  _cfg.buy_threshold);
    double sell_thr = RC().getd("prob_mom.sell_threshold", _cfg.sell_threshold);
    if (!(sell_thr < buy_thr)) {
        LOG_WARN("ProbabilityMomentumFactor: 阈值配置异常 (sell_threshold={} >= buy_threshold={})，使用默认 0.3/0.7",
                 sell_thr, buy_thr);
        sell_thr = 0.3;
        buy_thr  = 0.7;
    }
    _cfg.buy_threshold  = buy_thr;
    _cfg.sell_threshold = sell_thr;
}

// =====================[ topic 注册 ]=====================

void ProbabilityMomentumFactor::register_topics(size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_PROB_MOM_PROB,   capacity);
    bus.register_topic<double>(TOP_PROB_MOM_SIGNAL, capacity);
}

// =====================[ 内部辅助 ]=====================

void ProbabilityMomentumFactor::ensure_state(const std::string& code) {
    auto it = _states.find(code);
    if (it == _states.end()) {
        CodeState st;
        st.stats.reset(static_cast<std::size_t>(_cfg.window_size));
        _states.emplace(code, std::move(st));
    }
}

void ProbabilityMomentumFactor::on_code_added(const std::string& code) {
    ensure_state(code);
}

void ProbabilityMomentumFactor::compute_and_publish(
        const std::string& code,
        CodeState& S,
        int64_t ts_ms) {

    double mu = 0.0;
    double sigma = 0.0;

    if (!S.stats.mean_var(mu, sigma)) {
        return; // 样本不足或权重为 0
    }

    if (!(S.stats.sum_w() >= _cfg.min_total_weight)) {
        return; // 有效权重不足
    }

    if (!(sigma > 0.0)) {
        sigma = _cfg.min_sigma;
    }
    if (!(sigma > 0.0)) {
        LOG_DEBUG("ProbabilityMomentumFactor: sigma 非正, code={}", code);
        return;
    }

    // 正态分布下 P(R > 0) = Φ(mu / sigma)
    double z = mu / sigma;
    if (!std::isfinite(z)) {
        LOG_DEBUG("ProbabilityMomentumFactor: 非有限 z, code={}, z={}", code, z);
        return;
    }

    double F = Distributions::normal_cdf(z);
    if (!std::isfinite(F)) {
        LOG_DEBUG("ProbabilityMomentumFactor: 非有限 F, code={}, F={}", code, F);
        return;
    }

    // 生成信号：-1, 0, +1
    double signal = 0.0;
    if (F >= _cfg.buy_threshold) {
        signal = 1.0;
    } else if (F <= _cfg.sell_threshold) {
        signal = -1.0;
    }

    // 发布
    safe_publish<double>(TOP_PROB_MOM_PROB,   code, ts_ms, F);
    safe_publish<double>(TOP_PROB_MOM_SIGNAL, code, ts_ms, signal);
}

// 统一价格事件入口
void ProbabilityMomentumFactor::on_price_event(
        const std::string& code,
        int64_t ts_ms,
        double price,
        double volume) {

    if (!(price > 0.0)) return;

    ensure_state(code);
    auto& S = _states[code];

    if (!S.has_last_price) {
        S.has_last_price = true;
        S.last_price     = price;
        return;
    }

    if (!(S.last_price > 0.0) || !std::isfinite(S.last_price)) {
        S.last_price = price;
        return;
    }

    // 对数收益率，使用通用工具
    double r = NumericUtils<double>::log_return(price, S.last_price);
    if (!std::isfinite(r)) {
        S.last_price = price;
        return;
    }

    S.last_price = price;

    // 成交量权重。为防止 0 影响，设个最小权重 1.0
    double w = (volume > 0.0 ? volume : 1.0);

    S.stats.push(r, w);
    compute_and_publish(code, S, ts_ms);
}

// =====================[ 事件入口 ]=====================

void ProbabilityMomentumFactor::on_quote(const QuoteDepth& q) {
    double bid = q.bid_price;
    double ask = q.ask_price;
    if (!(bid > 0.0) || !(ask > 0.0)) return;

    double mid = 0.5 * (bid + ask);
    if (!std::isfinite(mid) || mid <= 0.0) return;

    double vol = static_cast<double>(q.volume);
    on_price_event(q.instrument_id, q.data_time_ms, mid, vol);
}

void ProbabilityMomentumFactor::on_tick(const CombinedTick& x) {
    if (!(x.price > 0.0)) return;

    double vol = static_cast<double>(x.volume);
    on_price_event(x.instrument_id, x.data_time_ms, x.price, vol);
}

void ProbabilityMomentumFactor::on_bar(const Bar& b) {
    if (!(b.close > 0.0)) return;

    double vol = static_cast<double>(b.volume);
    on_price_event(b.instrument_id, b.data_time_ms, b.close, vol);
}

} // namespace factorlib
