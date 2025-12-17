#include "factors/kline/hmm2_logodds_factor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace factorlib {

/*
（Hmm2LogOddsFactor）

- 输出 topic：`kline/hmm2_logodds`
- 含义：两状态（上行/下行）高斯 HMM 的上行后验概率 $\gamma_t$ 的 logit：
  $F_t=\log(\gamma_t/(1-\gamma_t))$。值越大，上行状态越“确定”；越小越偏下行。
- 典型场景：
  - 单边上涨：收益序列连续为正且波动稳定，$\gamma_t$ 上升，因子值转正并走高。
  - 单边下跌：$\gamma_t$ 下降，因子值转负并走低。
  - 震荡市：$\gamma_t$ 在 0.5 附近来回，因子值围绕 0 波动。
- 参数提示：`window_size` 越大越稳但滞后；`sigma_up/sigma_down` 过小会导致状态切换更“极端”。
*/

namespace {

constexpr const char* TOP_HMM2_LOGODDS = "kline/hmm2_logodds";

double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

double logit(double p) {
    p = clamp(p, 1e-12, 1.0 - 1e-12);
    return std::log(p) - std::log1p(-p);
}

void validate_config(const Hmm2LogOddsConfig& cfg) {
    if (cfg.window_size <= 0) {
        throw std::invalid_argument("Hmm2LogOddsConfig.window_size must be > 0");
    }
    if (!(cfg.sigma_up > 0.0) || !std::isfinite(cfg.sigma_up)) {
        throw std::invalid_argument("Hmm2LogOddsConfig.sigma_up must be finite and > 0");
    }
    if (!(cfg.sigma_down > 0.0) || !std::isfinite(cfg.sigma_down)) {
        throw std::invalid_argument("Hmm2LogOddsConfig.sigma_down must be finite and > 0");
    }
}

} // namespace

Hmm2LogOddsFactor::CodeState::CodeState(const Hmm2LogOddsConfig& cfg)
    : hmm(static_cast<std::size_t>(std::max(2, cfg.window_size)), cfg.to_math_params()) {}

void Hmm2LogOddsFactor::CodeState::push_bar(const Bar& b, const Hmm2LogOddsConfig& cfg) {
    if (!(b.close > 0.0)) {
        ++bad_close_count;
        return;
    }
    const double close = static_cast<double>(b.close);
    if (!has_last_close) {
        last_close = close;
        has_last_close = true;
        return;
    }

    double r = std::log(close / last_close);
    last_close = close;
    r = clamp(r, -cfg.r_cap, cfg.r_cap);
    (void)hmm.push(r);
}

double Hmm2LogOddsFactor::CodeState::log_odds() const {
    const double p = hmm.uptrend_prob();
    if (!std::isfinite(p)) return std::numeric_limits<double>::quiet_NaN();
    return logit(p);
}

Hmm2LogOddsFactor::Hmm2LogOddsFactor(const std::vector<Code>& codes, const Hmm2LogOddsConfig& cfg)
    : BaseFactor("Hmm2LogOddsFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {
    validate_config(_cfg);
}

bool Hmm2LogOddsFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void Hmm2LogOddsFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_HMM2_LOGODDS, capacity);
}

void Hmm2LogOddsFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg)).first;
    }
    CodeState& st = it->second;

    st.push_bar(b, _cfg);
    if (!st.ready()) return;

    double v = st.log_odds();
    if (!std::isfinite(v)) return;
    v = clamp(v, -50.0, 50.0);
    safe_publish<double>(TOP_HMM2_LOGODDS, b.instrument_id, b.data_time_ms, v);
}

} // namespace factorlib
