#include "factors/kline/gls_drift_z_factor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace factorlib {

/*
（WlsDriftZFactor；历史命名为 GlsDriftZFactor）

- 输出 topic：`kline/gls_drift_z`
- 含义：该实现本质是 WLS（按方差倒数加权的均值/漂移统计），并非严格 GLS。
  对收益 $r_t$ 做“按波动倒数加权”的漂移统计，输出近似 z-score：
  $F_t=\frac{\sum w_i r_i}{\sqrt{\sum w_i}}$，其中 $w_i \approx 1/\sigma_i^2$（$\sigma_i^2$ 用 Parkinson 区间波动估计）。
- 典型场景：
  - 低波动稳步上涨：$w_i$ 较大且 $r_i$ 多为正，$F_t$ 更容易显著为正。
  - 高波动但无方向：$r_i$ 正负抵消，$F_t$ 接近 0。
  - 持续下跌：$F_t$ 显著为负。
- 参数提示：`r_cap` 防止极端收益主导；`sigma2_floor` 防止 $w_i$ 过大；`min_obs` 控制最小样本数。
*/

namespace {

constexpr const char* TOP_GLS_DRIFT_Z = "kline/gls_drift_z";

double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

double parkinson_sigma2(const Bar& b) {
    if (!(b.high > 0.0) || !(b.low > 0.0)) return std::numeric_limits<double>::quiet_NaN();
    const double x = std::log(static_cast<double>(b.high) / static_cast<double>(b.low));
    const double denom = 4.0 * std::log(2.0);
    return (denom > 0.0) ? (x * x / denom) : std::numeric_limits<double>::quiet_NaN();
}

void validate_config(const GlsDriftZConfig& cfg) {
    if (cfg.window_size <= 0) {
        throw std::invalid_argument("GlsDriftZConfig.window_size must be > 0");
    }
    if (cfg.min_obs <= 0) {
        throw std::invalid_argument("GlsDriftZConfig.min_obs must be > 0");
    }
    if (cfg.min_obs > cfg.window_size) {
        throw std::invalid_argument("GlsDriftZConfig.min_obs must be <= window_size");
    }
    if (!(cfg.sigma2_floor > 0.0) || !std::isfinite(cfg.sigma2_floor)) {
        throw std::invalid_argument("GlsDriftZConfig.sigma2_floor must be finite and > 0");
    }
}

} // namespace

GlsDriftZFactor::CodeState::CodeState(const GlsDriftZConfig& cfg) {
    (void)cfg;
}

double GlsDriftZFactor::CodeState::value() const {
    if (!std::isfinite(sum_w) || !std::isfinite(sum_wr)) return std::numeric_limits<double>::quiet_NaN();
    if (!(sum_w > 0.0)) return std::numeric_limits<double>::quiet_NaN();
    return sum_wr / std::sqrt(sum_w);
}

bool GlsDriftZFactor::CodeState::push_bar(const Bar& b, const GlsDriftZConfig& cfg) {
    if (!(b.close > 0.0)) {
        ++bad_close_count;
        return false;
    }
    const double close = static_cast<double>(b.close);
    if (!has_last_close) {
        last_close = close;
        has_last_close = true;
        return false;
    }

    double r = std::log(close / last_close);
    last_close = close;
    r = clamp(r, -cfg.r_cap, cfg.r_cap);

    double s2 = parkinson_sigma2(b);
    if (!std::isfinite(s2)) {
        // 缺少 high/low 时：跳过权重样本，但仍已推进 last_close（收益链路不中断）。
        ++bad_hilo_count;
        return false;
    }
    s2 = std::max(s2, cfg.sigma2_floor);
    const double w = 1.0 / s2;
    if (!std::isfinite(w) || !(w > 0.0)) {
        ++bad_weight_count;
        return false;
    }
    const double wr = w * r;
    if (!std::isfinite(wr)) {
        ++bad_weight_count;
        return false;
    }

    window.push_back(Sample{w, wr});
    sum_w += w;
    sum_wr += wr;

    const int W = std::max(0, cfg.window_size);
    while (W > 0 && static_cast<int>(window.size()) > W) {
        const Sample old = window.front();
        window.pop_front();
        sum_w -= old.w;
        sum_wr -= old.wr;
    }

    return ready(cfg);
}

GlsDriftZFactor::GlsDriftZFactor(const std::vector<Code>& codes, const GlsDriftZConfig& cfg)
    : BaseFactor("GlsDriftZFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {
    validate_config(_cfg);
}

bool GlsDriftZFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void GlsDriftZFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_GLS_DRIFT_Z, capacity);
}

void GlsDriftZFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg)).first;
    }
    CodeState& st = it->second;

    if (!st.push_bar(b, _cfg)) return;

    double v = st.value();
    if (!std::isfinite(v)) return;
    v = clamp(v, -50.0, 50.0);
    safe_publish<double>(TOP_GLS_DRIFT_Z, b.instrument_id, b.data_time_ms, v);
}

} // namespace factorlib
