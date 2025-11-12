#include "granger_causality_factor.h"

#include <algorithm>
#include <cassert>
#include <Eigen/Dense>

#include "utils/trading_time.h"

namespace factorlib {

using Vec = Eigen::VectorXd;

// ===== 构造 / 注册 =====
GrangerCausalityFactor::GrangerCausalityFactor(const std::vector<std::string>& codes,
                                               const GrangerConfig& cfg)
    : BaseFactor("GrangerCausalityStrength", codes)
    , _cfg(cfg) {
}

void GrangerCausalityFactor::register_topics(size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_GRANGER_STRENGTH, capacity);
    bus.register_topic<double>(TOP_GRANGER_PVAL,     capacity); // 仅注册，不强制发布
}

// ===== BaseFactor 钩子：首次见到某个 code =====
void GrangerCausalityFactor::on_code_added(const std::string& code) {
    ensure_code_(code);
}

// ===== IFactor 事件入口 =====
void GrangerCausalityFactor::on_quote(const QuoteDepth& q) {
    on_any_event_(q.instrument_id, q.data_time_ms, q, std::nullopt, std::nullopt);
}
void GrangerCausalityFactor::on_transaction(const Transaction& t) {
    on_any_event_(t.instrument_id, t.data_time_ms, std::nullopt, t, std::nullopt);
}
void GrangerCausalityFactor::on_entrust(const Entrust& e) {
    on_any_event_(e.instrument_id, e.data_time_ms, std::nullopt, std::nullopt, e);
}

// ===== 强制冲桶（仅聚合模式有意义） =====
bool GrangerCausalityFactor::force_flush(const std::string& code) {
    auto it = _states.find(code);
    if (it == _states.end()) return false;
    if (_cfg.feed_mode == config::FeedMode::Aggregated) {
        BucketOutputs out{};
        if (it->second.bucket.force_flush(out)) {
            close_bucket_and_push_(code, out);
            it->second.last_bucket_ts = out.bucket_end_ms;
            return true;
        }
        return false;
    }
    return false;
}

// ===== 内部：确保 code 状态 =====
void GrangerCausalityFactor::ensure_code_(const std::string& code) {
    if (_states.find(code) != _states.end()) return;

    CodeState st;
    const int p = std::max(0, _cfg.p_lags);
    const int q = std::max(0, _cfg.q_lags);
    st.d_r = 1 + p;
    st.d_u = 1 + p + q;
    st.ne_r = math::SlidingNormalEq<double>(st.d_r, _cfg.window_size);
    st.ne_u = math::SlidingNormalEq<double>(st.d_u, _cfg.window_size);

    st.bucket.set_bucket_ms(_cfg.bucket_ms);

    _states.emplace(code, std::move(st));
}

// ===== 内部：统一事件入口（分为 聚合 / 逐条） =====
void GrangerCausalityFactor::on_any_event_(const std::string& code, int64_t ts_ms,
                                           const std::optional<QuoteDepth>& qopt,
                                           const std::optional<Transaction>& topt,
                                           const std::optional<Entrust>& eopt) {
    ensure_code_(code);
    auto& S = _states[code];

    // ---------- 路径一：聚合模式 ----------
    if (_cfg.feed_mode == config::FeedMode::Aggregated) {
        BucketOutputs out{};
        if (S.bucket.ensure_bucket(ts_ms, out)) {
            close_bucket_and_push_(code, out);
            S.last_bucket_ts = out.bucket_end_ms;
        }
        if (qopt) S.bucket.on_quote(*qopt);
        if (topt) S.bucket.on_transaction(*topt);
        if (eopt) S.bucket.on_entrust(*eopt);
        return;
    }

    // ---------- 路径二：逐条模式 ----------
    if (eopt) {
        const auto& e = *eopt;
        S.pending_ofi += (e.side > 0 ? (double)e.volume : -(double)e.volume);
    }

    if (topt) { /* 可选：基于成交方向累计 OFI */ }

    if (qopt) {
        const auto& q = *qopt;
        if (q.bid_price > 0 && q.ask_price > 0) {
            double mid_now = 0.5 * (q.bid_price + q.ask_price);
            if (S.last_mid.has_value()) {
                double y_t = mid_now - *S.last_mid;
                emit_sample_event_driven_(code, ts_ms, y_t);
            }
            S.last_mid = mid_now;
        }
    }
}

// ===== 内部：聚合模式收拢桶 -> 产出样本 =====
void GrangerCausalityFactor::close_bucket_and_push_(const std::string& code,
                                                    const BucketOutputs& bkt) {
    auto& S = _states[code];

    double mid_now = bkt.midprice_last;
    if (!(mid_now > 0)) return;

    double y_t = 0.0;
    if (S.last_mid.has_value()) y_t = mid_now - *S.last_mid;
    S.last_mid = mid_now;

    long long buy_vol = 0, sell_vol = 0;
    for (const auto& ord : bkt.orders) {
        if (ord.side > 0) buy_vol += (long long)ord.volume;
        else              sell_vol += (long long)ord.volume;
    }
    double x_t = (double)(buy_vol - sell_vol);

    S.x_win.push_back(x_t);
    S.y_win.push_back(y_t);
    while ((int)S.x_win.size() > _cfg.window_size) S.x_win.pop_front();
    while ((int)S.y_win.size() > _cfg.window_size) S.y_win.pop_front();

    if ((int)S.y_win.size() <= std::max(_cfg.p_lags, _cfg.q_lags)) return;
    std::vector<double> y_lags(_cfg.p_lags), x_lags(_cfg.q_lags);
    for (int i=0;i<_cfg.p_lags;++i) y_lags[i] = S.y_win[(int)S.y_win.size()-1 - (i+1)];
    for (int j=0;j<_cfg.q_lags;++j) x_lags[j] = S.x_win[(int)S.x_win.size()-1 - (j+1)];

    push_sample_and_update_(code, bkt.bucket_end_ms, y_t, y_lags, x_lags);
}

// ===== 内部：逐条模式在 mid 刷新时产出样本 =====
void GrangerCausalityFactor::emit_sample_event_driven_(const std::string& code,
                                                       int64_t ts_ms, double y_t) {
    auto& S = _states[code];

    double x_t = S.pending_ofi;
    S.pending_ofi = 0.0;

    S.x_win.push_back(x_t);
    S.y_win.push_back(y_t);
    while ((int)S.x_win.size() > _cfg.window_size) S.x_win.pop_front();
    while ((int)S.y_win.size() > _cfg.window_size) S.y_win.pop_front();

    if ((int)S.y_win.size() <= std::max(_cfg.p_lags, _cfg.q_lags)) return;
    std::vector<double> y_lags(_cfg.p_lags), x_lags(_cfg.q_lags);
    for (int i=0;i<_cfg.p_lags;++i) y_lags[i] = S.y_win[(int)S.y_win.size()-1 - (i+1)];
    for (int j=0;j<_cfg.q_lags;++j) x_lags[j] = S.x_win[(int)S.x_win.size()-1 - (j+1)];

    push_sample_and_update_(code, ts_ms, y_t, y_lags, x_lags);
}

// ===== 内部：核心回归 / 统计量 / 发布 =====
void GrangerCausalityFactor::push_sample_and_update_(const std::string& code, int64_t ts_ms,
                                                     double y_t,
                                                     const std::vector<double>& y_lags,
                                                     const std::vector<double>& x_lags) {
    auto& S = _states[code];
    const int p = _cfg.p_lags;
    const int q = _cfg.q_lags;

    Vec xr(S.d_r);
    xr(0) = 1.0;
    for (int i=0;i<p;++i) xr(1+i) = y_lags[i];

    Vec xu(S.d_u);
    xu(0) = 1.0;
    for (int i=0;i<p;++i) xu(1+i) = y_lags[i];
    for (int j=0;j<q;++j) xu(1+p+j) = x_lags[j];

    S.ne_r.push(xr, y_t);
    S.ne_u.push(xu, y_t);

    const int N   = std::min((int)S.y_win.size(), _cfg.window_size);
    const int df2 = N - (p + q + 1);
    if (N < std::max(_cfg.min_effective, p + q + 4) || df2 <= 0) return;

    Vec br(S.d_r), bu(S.d_u);
    double RSSr=0, RSSu=0;
    if (!S.ne_r.solve(br, RSSr)) return;
    if (!S.ne_u.solve(bu, RSSu)) return;

    const int k = std::max(1, q);
    double num = (RSSr - RSSu) / (double)k;
    if (num < 0) num = 0;
    double den = RSSu / (double)std::max(1, df2);
    if (den <= 0) return;
    const double F = num / den;

    double pval = math::fisher_f_sf<double>(F, k, df2);
    publish_strength_(code, ts_ms, pval);

    if (_cfg.debug_mode) {
        LOG_DEBUG("[{}] ts={} N={} F={} p={} RSSr={} RSSu={}",
                  code, ts_ms, N, F, pval, RSSr, RSSu);
    }
}

void GrangerCausalityFactor::publish_strength_(const std::string& code,
                                               int64_t ts_ms, double p) const {
    double value = p;
    double raw_p = p;
    if (_cfg.use_neglog10) {
        if (p <= 0) p = std::numeric_limits<double>::min();
        value = -std::log10(p);
        if (value > _cfg.strength_clip) value = _cfg.strength_clip;
        if (!std::isfinite(value)) value = 0.0;
    }
    safe_publish<double>(TOP_GRANGER_STRENGTH, code, ts_ms, value);
    if (_cfg.publish_raw_p) {
        safe_publish<double>(TOP_GRANGER_PVAL, code, ts_ms, raw_p);
    }
}

} // namespace factorlib
