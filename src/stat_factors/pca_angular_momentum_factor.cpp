#include "pca_angular_momentum_factor.h"

#include <algorithm>
#include <limits>

#include "../config/runtime_config.h"

using factorlib::config::RC;

namespace factorlib {

// =====================[ CodeState 实现 ]=====================

PcaAngularMomentumFactor::CodeState::CodeState(const PcaAngularMomentumConfig& cfg)
    : pca(cfg.dims, std::max(2, cfg.k), cfg.lr)
    , has_last_close(false)
    , last_close(0.0)
    , last_proj_valid(false)
    , last_u(0.0)
    , last_v(0.0)
    , n_samples(0) {}

// =====================[ 构造 & 配置 ]=====================

PcaAngularMomentumFactor::PcaAngularMomentumFactor(
        const std::vector<std::string>& codes,
        const PcaAngularMomentumConfig& cfg)
    : BaseFactor("PcaAngularMomentumFactor", codes)
    , _cfg(cfg) {

    _cfg.dims       = RC().geti("pca_ang.dims",       _cfg.dims);
    _cfg.k          = RC().geti("pca_ang.k",          _cfg.k);
    _cfg.warmup     = RC().geti("pca_ang.warmup",     _cfg.warmup);
    _cfg.lr         = RC().getd("pca_ang.lr",         _cfg.lr);

    if (_cfg.dims <= 0) _cfg.dims = 3;
    if (_cfg.k < 2) _cfg.k = 2;
    if (_cfg.k > _cfg.dims) _cfg.k = std::min(2, _cfg.dims);
    if (_cfg.lr <= 0.0) _cfg.lr = 0.05;
    if (_cfg.warmup < 8) _cfg.warmup = 8;
}

// =====================[ topic 注册 ]=====================

void PcaAngularMomentumFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_PCA_ANG, capacity);
}

// =====================[ state 管理 ]=====================

void PcaAngularMomentumFactor::ensure_state(const std::string& code) {
    if (_states.find(code) != _states.end()) return;
    _states.emplace(code, CodeState(_cfg));
}

void PcaAngularMomentumFactor::on_code_added(const std::string& code) {
    ensure_state(code);
}

// =====================[ 特征构造 ]=====================

std::vector<double> PcaAngularMomentumFactor::make_features(const Bar& b,
                                                            double ret) const {
    std::vector<double> x(static_cast<std::size_t>(_cfg.dims), 0.0);
    const double vol = static_cast<double>(b.volume);
    double vwap = b.close;
    if (vol > 0.0 && b.turnover > 0.0) {
        vwap = b.turnover / vol;
    }

    if (_cfg.dims >= 1) x[0] = ret;
    if (_cfg.dims >= 2) x[1] = std::log(vol + 1.0);
    if (_cfg.dims >= 3) x[2] = std::log(vwap > 0.0 ? vwap : b.close);

    return x;
}

// =====================[ 核心逻辑 ]=====================

void PcaAngularMomentumFactor::on_price_event(const std::string& code,
                                              int64_t ts_ms,
                                              const Bar& b) {
    if (!(b.close > 0.0)) return;

    ensure_state(code);
    auto& st = _states.at(code);

    double close = b.close;
    if (!st.has_last_close) {
        st.last_close = close;
        st.has_last_close = true;
        return;
    }

    double ret = 0.0;
    double ratio = close / st.last_close;
    if (ratio > 0.0) {
        ret = std::log(ratio);
    }
    st.last_close = close;

    if (!std::isfinite(ret)) return;

    auto x = make_features(b, ret);

    if (!st.pca.push(x)) {
        LOG_DEBUG("PcaAngularMomentumFactor: push failed, code={}", code);
        return;
    }

    ++st.n_samples;
    if (st.n_samples < _cfg.warmup) return;

    maybe_publish(code, st, ts_ms, x);
}

void PcaAngularMomentumFactor::maybe_publish(const std::string& code,
                                             CodeState& st,
                                             int64_t ts_ms,
                                             const std::vector<double>& x) {
    const auto& comps = st.pca.components();
    if (comps.size() < 2) return; // 需要至少 2 个主成分

    const auto& pc1 = comps[0];
    const auto& pc2 = comps[1];

    if (pc1.size() != x.size() || pc2.size() != x.size()) {
        return;
    }

    double u = 0.0, v = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        u += x[i] * pc1[i];
        v += x[i] * pc2[i];
    }

    if (!st.last_proj_valid) {
        st.last_u = u;
        st.last_v = v;
        st.last_proj_valid = true;
        return;
    }

    // 2D 叉积模长：| (u_{t-1}, v_{t-1}) x (u_t, v_t) |
    double L = std::fabs(st.last_u * v - st.last_v * u);

    st.last_u = u;
    st.last_v = v;
    st.last_proj_valid = true;

    LOG_DEBUG("PcaAngularMomentumFactor: code={} ts={} L={}", code, ts_ms, L);

    safe_publish<double>(TOP_PCA_ANG, code, ts_ms, L);
}

// =====================[ IFactor 接口 ]=====================

void PcaAngularMomentumFactor::on_bar(const Bar& b) {
    on_price_event(b.instrument_id, b.data_time_ms, b);
}

} // namespace factorlib
