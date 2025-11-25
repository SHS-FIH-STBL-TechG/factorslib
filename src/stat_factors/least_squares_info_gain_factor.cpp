#include "least_squares_info_gain_factor.h"

#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>

#include "utils/databus.h"
#include "utils/log.h"
#include "math/distributions.h"
#include "../config/runtime_config.h"
#include "utils/config_utils.h"

using factorlib::config::RC;

namespace factorlib {

LeastSquaresInfoGainFactor::LeastSquaresInfoGainFactor(
        const std::vector<std::string>& codes,
        const LsInfoGainConfig& cfg)
    : BaseFactor("LeastSquaresInfoGain", codes),
      _cfg(cfg) {

    int ws = RC().geti("lsig.window_size", _cfg.window_size);
    if (ws <= 0) {
        LOG_WARN("LeastSquaresInfoGainFactor: window_size={} 无效，重置为 30", ws);
        ws = 30;
    }
    _cfg.window_size = ws;
    _window_sizes = factorlib::config::load_window_sizes("lsig", _cfg.window_size);
    clamp_window_list(_window_sizes, "[lsig] window_sizes");
    auto freq_cfg = factorlib::config::load_time_frequencies("lsig");
    if (!freq_cfg.empty()) {
        clamp_frequency_list(freq_cfg, "[lsig] time_frequencies");
        set_time_frequencies_override(freq_cfg);
    }
}

void LeastSquaresInfoGainFactor::register_topics(size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_LSIG_IG,    capacity);
    bus.register_topic<double>(TOP_LSIG_IG_LO, capacity);
    bus.register_topic<double>(TOP_LSIG_IG_HI, capacity);
}

// ---------------- 内部：状态初始化 ----------------

LeastSquaresInfoGainFactor::CodeState& LeastSquaresInfoGainFactor::ensure_state(const ScopeKey& scope) {
    auto key = scope.as_bus_code();
    auto it = _states.find(key);
    if (it == _states.end()) {
        CodeState s;
        s.window_size = scope.window > 0 ? scope.window : _cfg.window_size;
        s.ols     = std::make_unique<SlidingOLS>(2, s.window_size);
        s.y_stats = std::make_unique<SlidingStats>(s.window_size);
        it = _states.emplace(std::move(key), std::move(s)).first;
        return it->second;
    }
    if (!it->second.ols || it->second.window_size != (scope.window > 0 ? scope.window : _cfg.window_size)) {
        it->second.window_size = scope.window > 0 ? scope.window : _cfg.window_size;
        it->second.ols = std::make_unique<SlidingOLS>(2, it->second.window_size);
        it->second.y_stats = std::make_unique<SlidingStats>(it->second.window_size);
    } else {
        if (!it->second.ols) {
            it->second.ols = std::make_unique<SlidingOLS>(2, it->second.window_size);
        }
        if (!it->second.y_stats) {
            it->second.y_stats = std::make_unique<SlidingStats>(it->second.window_size);
        }
    }
    return it->second;
}

void LeastSquaresInfoGainFactor::on_code_added(const std::string& code) {
    const auto& freqs = get_time_frequencies();
    for (int freq : freqs) {
        for (int window : _window_sizes) {
            (void)ensure_state(ScopeKey{code, freq, window});
        }
    }
}

    // 快照：用中间价
    void LeastSquaresInfoGainFactor::on_quote(const QuoteDepth& q) {
    double bid = q.bid_price;
    double ask = q.ask_price;
    if (!(bid > 0.0) || !(ask > 0.0)) return;
    double mid = 0.5 * (bid + ask);
    if (!std::isfinite(mid) || mid <= 0.0) return;
    on_price_event(q.instrument_id, q.data_time_ms, mid);
}

    // 逐笔（成交/委托都在 CombinedTick 里区分 kind）
    void LeastSquaresInfoGainFactor::on_tick(const CombinedTick& x) {
    if (!(x.price > 0.0)) return;
    on_price_event(x.instrument_id, x.data_time_ms, x.price);
}

    // K 线：用收盘价
    void LeastSquaresInfoGainFactor::on_bar(const Bar& b) {
    if (!(b.close > 0.0)) return;
    on_price_event(b.instrument_id, b.data_time_ms, b.close);
}


// ---------------- 统一价格入口：计算 r_t 并推样本 ----------------

void LeastSquaresInfoGainFactor::on_price_event(
        const std::string& code_raw,
        int64_t ts_ms,
        double px) {

    ensure_code(code_raw);
    for_each_scope(code_raw, _window_sizes, ts_ms, [&](const ScopeKey& scope) {
        CodeState& S = ensure_state(scope);
        const std::string scoped_code = scope.as_bus_code();

        // 第一次价格，只记录
        if (!S.has_last_px) {
            S.last_px = px;
            S.has_last_px = true;
            S.has_prev_ret = false;
            return;
        }

        // 非正价格：重置收益链
        if (!(px > 0.0) || !(S.last_px > 0.0)) {
            S.last_px     = px;
            S.has_prev_ret = false;
            return;
        }

        const double r_t = std::log(px / S.last_px);  // ★ ln(P_t / P_{t-1})
        S.last_px = px;

        if (!std::isfinite(r_t)) {
            S.has_prev_ret = false;
            return;
        }

        // 第一条 r_t：只存，不构造 (x,y)
        if (!S.has_prev_ret) {
            S.prev_ret    = r_t;
            S.has_prev_ret = true;
            return;
        }

        const double x_prev = S.prev_ret;  // r_{t-1}
        const double y_curr = r_t;         // r_t
        S.prev_ret = r_t;

        push_sample_and_update(S, scope, ts_ms, x_prev, y_curr);
    });
}

// ---------------- 推入样本并在满窗后计算因子 ----------------

void LeastSquaresInfoGainFactor::push_sample_and_update(
        CodeState& S,
        const ScopeKey& scope,
        int64_t ts_ms,
        double x_prev_ret,
        double y_curr_ret) {

    if (!S.ols || !S.y_stats) return;
    const std::string scoped_code = scope.as_bus_code();

    // 1) 推入滑动 OLS & y 统计
    Eigen::VectorXd x(2);
    x(0) = 1.0;          // 截距
    x(1) = x_prev_ret;

    S.ols->push(x, y_curr_ret);
    S.y_stats->push(y_curr_ret);

    const int n        = S.ols->size();
    const int win_size = S.window_size > 0 ? S.window_size : _cfg.window_size;

    // 窗口未满，不产出
    if (n < win_size) {
        return;
    }

    // 最少 3 条样本，避免自由度为 0
    if (n < 3) return;

    // 2) 原始 Y 方差：s0^2
    const double sse0 = S.y_stats->sse();
    const int df0 = n - 1;
    if (df0 <= 0) return;

    double s0_sq = sse0 / static_cast<double>(df0);

    // 3) 残差方差：s1^2 = RSS / (n - 2)
    Eigen::VectorXd beta(2);
    double RSS = 0.0;
    const bool ok = S.ols->solve(beta, RSS);
    if (!ok) {
        LOG_WARN("LeastSquaresInfoGainFactor: solve 失败 code={}, n={}", scoped_code, n);
        return;
    }

    const int df1 = n - 2;
    if (df1 <= 0) return;

    double s1_sq = RSS / static_cast<double>(df1);

    // 4) ln 方差比（信息增益）
    const double eps = 1e-12;
    if (!(s0_sq > 0.0)) s0_sq = eps;
    if (!(s1_sq > 0.0)) s1_sq = eps;

    const double R = s0_sq / s1_sq;
    if (!(R > 0.0)) return;

    const double ig = std::log(R);  // ★ 因子结果：ln(s0^2 / s1^2)

    // 5) 95% 置信区间
    double var_logR = 2.0 / static_cast<double>(df0)
                    + 2.0 / static_cast<double>(df1);
    if (var_logR < 0.0) var_logR = 0.0;
    const double se = std::sqrt(var_logR);

    const double z975 = math::Distributions::normal_quantile(0.975);
    const double ig_lo = ig - z975 * se;
    const double ig_hi = ig + z975 * se;

    LOG_DEBUG("LeastSquaresInfoGainFactor: code={} ts={} n={} s0^2={} s1^2={} R={} ig={} lo={} hi={}",
              scoped_code, ts_ms, n, s0_sq, s1_sq, R, ig, ig_lo, ig_hi);

    publish_all(scoped_code, ts_ms, ig, ig_lo, ig_hi);
}

// ---------------- 发布 ----------------

void LeastSquaresInfoGainFactor::publish_all(
        const std::string& code,
        int64_t ts_ms,
        double ig,
        double ig_lo,
        double ig_hi) {

    safe_publish<double>(TOP_LSIG_IG,    code, ts_ms, ig);
    safe_publish<double>(TOP_LSIG_IG_LO, code, ts_ms, ig_lo);
    safe_publish<double>(TOP_LSIG_IG_HI, code, ts_ms, ig_hi);
}

} // namespace factorlib
