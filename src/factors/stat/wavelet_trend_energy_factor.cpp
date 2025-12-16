#include "factors/stat/wavelet_trend_energy_factor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "config/runtime_config.h"
#include "config/config_utils.h"
#include "core/databus.h"
#include "utils/log.h"

using factorlib::config::RC;

namespace factorlib {

// =====================[ CodeState 实现 ]=====================

WaveletTrendEnergyFactor::CodeState::CodeState(const WaveTrendConfig& cfg)
    : modwt(static_cast<std::size_t>(cfg.window_size),
            cfg.levels_J,
            (cfg.wavelet == "sym4"
                 ? math::wavelet_sym4()
                 : math::wavelet_db4())) {
    window_size = cfg.window_size;
}

static inline double safe_log(double x) {
    return (x > 0.0) ? std::log(x) : std::numeric_limits<double>::quiet_NaN();
}

static inline double sign(double x) {
    return (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0);
}

// =====================[ 构造 & 配置 ]=====================

WaveletTrendEnergyFactor::WaveletTrendEnergyFactor(
        const std::vector<std::string>& codes,
        const WaveTrendConfig& cfg)
    : BaseFactor("WaveletTrendEnergy", codes)
    , _cfg(cfg) {

    // 运行时配置覆盖
    int ws = RC().geti("wave_trend.window_size", _cfg.window_size);
    if (ws < 8) {
        LOG_WARN("WaveletTrendEnergy: window_size={} 太小，重置为 128", ws);
        ws = 128;
    }
    _cfg.window_size = ws;

    int J = RC().geti("wave_trend.levels_J", _cfg.levels_J);
    if (J <= 0) J = 6;
    _cfg.levels_J = J;

    int jt = RC().geti("wave_trend.trend_start_j", _cfg.trend_start_j);
    if (jt < 1) jt = 1;
    if (jt > _cfg.levels_J) jt = _cfg.levels_J;
    _cfg.trend_start_j = jt;

    _cfg.wavelet    = RC().get("wave_trend.wavelet",    _cfg.wavelet);
    _window_sizes   = factorlib::config::load_window_sizes("wave_trend", _cfg.window_size);
    clamp_window_list(_window_sizes, "[wave_trend] window_sizes");
}

// =====================[ DataBus topic 注册 ]=====================

void WaveletTrendEnergyFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_WAVE_TREND, capacity);
}

// =====================[ 状态管理 ]=====================

WaveletTrendEnergyFactor::CodeState& WaveletTrendEnergyFactor::ensure_state(const ScopeKey& scope) {
    auto key = scope.as_bus_code();
    auto it = _states.find(key);
    if (it == _states.end()) {
        auto cfg = _cfg;
        cfg.window_size = scope.window > 0 ? scope.window : _cfg.window_size;
        it = _states.emplace(std::piecewise_construct,
                             std::forward_as_tuple(key),
                             std::forward_as_tuple(cfg)).first;
    }
    return it->second;
}

void WaveletTrendEnergyFactor::on_code_added(const std::string& code) {
    for (int window : _window_sizes) {
        (void)ensure_state(ScopeKey{code, window});
    }
}

// =====================[ 统一价格入口 ]=====================

void WaveletTrendEnergyFactor::on_price_event(const std::string& code_raw,
                                              int64_t ts_ms,
                                              double price) {
    if (!(price > 0.0)) return;

    const double logp = safe_log(price);
    if (!std::isfinite(logp)) return;

    ensure_code(code_raw);
    for_each_scope(code_raw, _window_sizes, ts_ms, [&](const ScopeKey& scope) {
        auto& st = ensure_state(scope);
        const std::string scoped_code = scope.as_bus_code();

        // Keep a log-price window aligned with the MODWT window of dx (need W+1 log prices for W diffs).
        const int W = std::max(1, st.window_size);
        st.log_price_window.push_back(logp);
        while (static_cast<int>(st.log_price_window.size()) > W + 1) {
            st.log_price_window.pop_front();
        }
        if (st.log_price_window.size() >= 2) {
            const double d = st.log_price_window.back() - st.log_price_window.front();
            if (std::isfinite(d) && std::abs(d) > 1e-12) {
                const double s = sign(d);
                if (s != 0.0) st.last_dir = s;
            }
        }

        if (!st.has_last_logp) {
            st.last_logp = logp;
            st.has_last_logp = true;
            return;
        }
        const double dx = logp - st.last_logp;
        st.last_logp = logp;
        if (!std::isfinite(dx)) {
            return;
        }
        if (std::abs(dx) <= 1e-12) {
            ++st.consecutive_zero_dx;
        } else {
            st.consecutive_zero_dx = 0;
        }

        if (!st.modwt.push(dx)) {
            LOG_DEBUG("WaveletTrendEnergy: push failed, code={}, log_price={}", scoped_code, logp);
            return;
        }

        if (st.modwt.ready()) {
            compute_and_publish(scoped_code, st, ts_ms);
        }
    });
}

void WaveletTrendEnergyFactor::compute_and_publish(const std::string& code,
                                                   CodeState& st,
                                                   int64_t ts_ms) {
    const double raw = st.modwt.trend_energy_ratio(_cfg.trend_start_j);
    double ratio = 0.0;
    if (!std::isfinite(raw)) {
        if (st.consecutive_zero_dx >= std::max(1, st.window_size)) {
            ratio = 1.0;
        } else {
            return;
        }
    } else {
        ratio = std::clamp(raw, 0.0, 1.0);
    }

    const double signed_ratio = std::clamp(st.last_dir * ratio, -1.0, 1.0);

    LOG_DEBUG("WaveletTrendEnergy: code={} ts={} ratio={} dir={} signed={}",
              code, ts_ms, ratio, st.last_dir, signed_ratio);

    safe_publish<double>(TOP_WAVE_TREND, code, ts_ms, signed_ratio);
}

// =====================[ IFactor 接口 ]=====================

void WaveletTrendEnergyFactor::on_quote(const QuoteDepth& q) {
    // 只在 bid/ask 都有效时用中间价
    if (q.bid_price <= 0.0 || q.ask_price <= 0.0) return;
    double mid = 0.5 * (q.bid_price + q.ask_price);
    on_price_event(q.instrument_id, q.data_time_ms, mid);
}

void WaveletTrendEnergyFactor::on_tick(const CombinedTick& x) {
    double price = x.price;
    if (price <= 0.0) return;
    on_price_event(x.instrument_id, x.data_time_ms, price);
}

void WaveletTrendEnergyFactor::on_bar(const Bar& b) {
    double price = b.close;
    if (price <= 0.0) return;
    on_price_event(b.instrument_id, b.data_time_ms, price);
}

} // namespace factorlib
