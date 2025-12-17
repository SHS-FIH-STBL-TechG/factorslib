#include "factors/kline/bayes_drift_logodds_factor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "math/distributions.h"

namespace factorlib {

/*
（BayesDriftLogOddsFactor）

- 输出 topic：`kline/bayes_drift_logodds`
- 含义：在滑窗内用“带方差权重”的贝叶斯更新估计收益漂移 $\mu$，输出 $P(\mu>0)$ 的 logit。
  直觉上：近期加权平均收益越偏正，值越大；越偏负，值越小。
- 典型场景：
  - 上涨但波动较大：若收益持续为正，即使波动大，后验仍可能显著偏正（输出为正）。
  - 下跌行情：后验偏负（输出为负）。
  - 缺失 high/low：Parkinson 波动不可算时会跳过该样本权重更新，可能导致当日不输出。
- 参数提示：`sigma2_floor` 用于避免权重爆炸；`tau0` 控制先验强度（越大越“相信数据”）。
*/

namespace {

constexpr const char* TOP_BAYES_DRIFT_LOGODDS = "kline/bayes_drift_logodds";

double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

double logit(double p) {
    p = clamp(p, 1e-12, 1.0 - 1e-12);
    return std::log(p) - std::log1p(-p);
}

double parkinson_sigma2(const Bar& b) {
    if (!(b.high > 0.0) || !(b.low > 0.0)) return std::numeric_limits<double>::quiet_NaN();
    const double x = std::log(static_cast<double>(b.high) / static_cast<double>(b.low));
    const double denom = 4.0 * std::log(2.0);
    return (denom > 0.0) ? (x * x / denom) : std::numeric_limits<double>::quiet_NaN();
}

void validate_config(const BayesDriftLogOddsConfig& cfg) {
    if (cfg.window_size <= 0) {
        throw std::invalid_argument("BayesDriftLogOddsConfig.window_size must be > 0");
    }
    if (cfg.min_obs <= 0) {
        throw std::invalid_argument("BayesDriftLogOddsConfig.min_obs must be > 0");
    }
    if (cfg.min_obs > cfg.window_size) {
        throw std::invalid_argument("BayesDriftLogOddsConfig.min_obs must be <= window_size");
    }
    if (!(cfg.sigma2_floor > 0.0) || !std::isfinite(cfg.sigma2_floor)) {
        throw std::invalid_argument("BayesDriftLogOddsConfig.sigma2_floor must be finite and > 0");
    }
    if (!(cfg.tau0 > 0.0) || !std::isfinite(cfg.tau0)) {
        throw std::invalid_argument("BayesDriftLogOddsConfig.tau0 must be finite and > 0");
    }
}

} // namespace

BayesDriftLogOddsFactor::CodeState::CodeState(const BayesDriftLogOddsConfig& cfg) {
    (void)cfg;
}

bool BayesDriftLogOddsFactor::CodeState::push_bar(const Bar& b, const BayesDriftLogOddsConfig& cfg) {
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

double BayesDriftLogOddsFactor::CodeState::value(const BayesDriftLogOddsConfig& cfg) const {
    if (!ready(cfg)) return std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(cfg.mu0) || !std::isfinite(sum_w) || !std::isfinite(sum_wr)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (!(sum_w > 0.0)) return std::numeric_limits<double>::quiet_NaN();

    const double prior_prec = 1.0 / (cfg.tau0 * cfg.tau0);
    const double post_prec = prior_prec + sum_w;
    if (!(post_prec > 0.0)) return std::numeric_limits<double>::quiet_NaN();

    const double post_mean = (prior_prec * cfg.mu0 + sum_wr) / post_prec;
    const double post_sd = 1.0 / std::sqrt(post_prec);
    if (!(post_sd > 0.0) || !std::isfinite(post_mean) || !std::isfinite(post_sd)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double z = post_mean / post_sd;
    const double p = math::Distributions<>::normal_cdf(z);
    return logit(p);
}

BayesDriftLogOddsFactor::BayesDriftLogOddsFactor(const std::vector<Code>& codes,
                                                 const BayesDriftLogOddsConfig& cfg)
    : BaseFactor("BayesDriftLogOddsFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {
    validate_config(_cfg);
}

bool BayesDriftLogOddsFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void BayesDriftLogOddsFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_BAYES_DRIFT_LOGODDS, capacity);
}

void BayesDriftLogOddsFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg)).first;
    }
    CodeState& st = it->second;

    if (!st.push_bar(b, _cfg)) return;

    double v = st.value(_cfg);
    if (!std::isfinite(v)) return;
    v = clamp(v, -50.0, 50.0);
    safe_publish<double>(TOP_BAYES_DRIFT_LOGODDS, b.instrument_id, b.data_time_ms, v);
}

} // namespace factorlib
