#include "pca_structure_stability_factor.h"

#include <algorithm>
#include <limits>

#include "../config/runtime_config.h"
#include "utils/config_utils.h"

using factorlib::config::RC;

namespace factorlib {

// =====================[ CodeState 实现 ]=====================

PcaStructureStabilityFactor::CodeState::CodeState(const PcaStructureStabilityConfig& cfg)
    : pca(cfg.dims, cfg.k, cfg.lr)
    , has_last_close(false)
    , last_close(0.0)
    , last_pc1_valid(false)
    , last_pc1(static_cast<std::size_t>(cfg.dims), 0.0)
    , n_samples(0) {}

// =====================[ 构造 & 配置 ]=====================

PcaStructureStabilityFactor::PcaStructureStabilityFactor(
        const std::vector<std::string>& codes,
        const PcaStructureStabilityConfig& cfg)
    : BaseFactor("PcaStructureStabilityFactor", codes)
    , _cfg(cfg) {

    _cfg.dims       = RC().geti("pca_stab.dims",       _cfg.dims);
    _cfg.k          = RC().geti("pca_stab.k",          _cfg.k);
    _cfg.warmup     = RC().geti("pca_stab.warmup",     _cfg.warmup);
    _cfg.lr         = RC().getd("pca_stab.lr",         _cfg.lr);

    if (_cfg.dims <= 0) _cfg.dims = 3;
    if (_cfg.k <= 0 || _cfg.k > _cfg.dims) _cfg.k = 1;
    if (_cfg.lr <= 0.0) _cfg.lr = 0.05;
    if (_cfg.warmup < 8) _cfg.warmup = 8;
    _window_sizes = {0};
    auto freq_cfg = factorlib::config::load_time_frequencies("pca_stab");
    if (!freq_cfg.empty()) set_time_frequencies_override(freq_cfg);
}

// =====================[ topic 注册 ]=====================

void PcaStructureStabilityFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_PCA_STAB, capacity);
}

// =====================[ state 管理 ]=====================

PcaStructureStabilityFactor::CodeState& PcaStructureStabilityFactor::ensure_state(const ScopeKey& scope) {
    auto key = scope.as_bus_code();
    auto it = _states.find(key);
    if (it == _states.end()) {
        it = _states.emplace(key, CodeState(_cfg)).first;
    }
    return it->second;
}

void PcaStructureStabilityFactor::on_code_added(const std::string& code) {
    const auto& freqs = get_time_frequencies();
    for (int freq : freqs) {
        for (int window : _window_sizes) {
            (void)ensure_state(ScopeKey{code, freq, window});
        }
    }
}

// =====================[ 特征构造 ]=====================

std::vector<double> PcaStructureStabilityFactor::make_features(const Bar& b,
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

    // 额外维度若存在，保持 0 即可（当前不使用）
    return x;
}

// =====================[ 核心逻辑 ]=====================

void PcaStructureStabilityFactor::on_price_event(const std::string& code_raw,
                                                 int64_t ts_ms,
                                                 const Bar& b) {
    if (!(b.close > 0.0)) return;

    ensure_code(code_raw);
    for_each_scope(code_raw, _window_sizes, [&](const ScopeKey& scope) {
        auto& st = ensure_state(scope);
        const std::string scoped_code = scope.as_bus_code();

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
            LOG_DEBUG("PcaStructureStabilityFactor: push failed, code={}", scoped_code);
            return;
        }

        ++st.n_samples;
        if (st.n_samples < _cfg.warmup) return;

        maybe_publish(scoped_code, st, ts_ms);
    });
}

void PcaStructureStabilityFactor::maybe_publish(const std::string& code,
                                                CodeState& st,
                                                int64_t ts_ms) {
    const auto& comps = st.pca.components();
    if (comps.empty()) return;
    const auto& pc1 = comps.front();
    if (pc1.size() != st.last_pc1.size()) {
        // 维度变化（理论上不会发生），重置
        st.last_pc1.assign(pc1.size(), 0.0);
        st.last_pc1_valid = false;
    }

    double cos_val = 0.0;
    if (st.last_pc1_valid) {
        double dot = 0.0, n1 = 0.0, n2 = 0.0;
        for (std::size_t i = 0; i < pc1.size(); ++i) {
            double a = pc1[i];
            double b = st.last_pc1[i];
            dot += a * b;
            n1  += a * a;
            n2  += b * b;
        }
        if (n1 > 0.0 && n2 > 0.0) {
            cos_val = dot / (std::sqrt(n1) * std::sqrt(n2));
            if (cos_val > 1.0) cos_val = 1.0;
            if (cos_val < -1.0) cos_val = -1.0;
        } else {
            cos_val = 0.0;
        }
    } else {
        // 第一次有 pc1，暂时不给出稳定性评分
        st.last_pc1 = pc1;
        st.last_pc1_valid = true;
        return;
    }

    st.last_pc1 = pc1;
    st.last_pc1_valid = true;

    double stability = std::fabs(cos_val);

    LOG_DEBUG("PcaStructureStabilityFactor: code={} ts={} stability={} cos={}",
              code, ts_ms, stability, cos_val);

    safe_publish<double>(TOP_PCA_STAB, code, ts_ms, stability);
}

// =====================[ IFactor 接口 ]=====================

void PcaStructureStabilityFactor::on_bar(const Bar& b) {
    on_price_event(b.instrument_id, b.data_time_ms, b);
}

} // namespace factorlib
