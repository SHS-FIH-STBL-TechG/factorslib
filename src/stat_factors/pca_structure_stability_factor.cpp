#include "pca_structure_stability_factor.h"
#include <algorithm>

using factorlib::config::RC;

namespace factorlib {

PCAStructureStabilityFactor::PCAStructureStabilityFactor(const std::vector<std::string>& codes,
                                                         const PCAStabilityCfg& cfg)
    : BaseFactor("PCAStability", codes), _cfg(cfg) {
    _cfg.dims = RC().geti("pca_stab.dims", _cfg.dims);
    _cfg.k    = RC().geti("pca_stab.k",    _cfg.k);
    _cfg.lr   = RC().getd("pca_stab.lr",   _cfg.lr);
    _cfg.debug_mode = RC().getb("pca_stab.debug_mode", _cfg.debug_mode);
}

void PCAStructureStabilityFactor::ensure_state(const std::string& code) {
    if (_states.find(code)!=_states.end()) return;
    CodeState st;
    st.pca = math::OnlinePCA(_cfg.dims, std::max(1,_cfg.k), _cfg.lr);
    _states.emplace(code, std::move(st));
}

void PCAStructureStabilityFactor::feed_vec(const std::string& code, int64_t ts, const std::vector<double>& x) {
    (void)ts;
    ensure_state(code);
    auto& s = _states[code];
    s.pca.push(x);
    maybe_publish(code, ts);
}

void PCAStructureStabilityFactor::maybe_publish(const std::string& code, int64_t ts) {
    auto& s = _states[code];
    if (s.pca.n_samples() < 4) return;
    const auto& comps = s.pca.components();
    if (comps.empty()) return;
    const auto& v = comps[0];

    // 计算与上一帧的 cos 相似度
    double cos_sim = std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(s.last_v1_angle)) {
        // last_v1_angle 存储上一步与参考向量的夹角的 cos 值，
        // 为简化我们存储上一步的主方向向量，并在这里与当前做点积。
    }
    // 我们直接存储上一主向量
    static thread_local std::unordered_map<std::string, std::vector<double>> last_vecs;
    auto it = last_vecs.find(code);
    if (it != last_vecs.end()) {
        const auto& u = it->second;
        double dot=0.0, nu=0.0, nv=0.0;
        for (size_t i=0;i<v.size();++i){ dot += u[i]*v[i]; nu += u[i]*u[i]; nv += v[i]*v[i]; }
        if (nu>0 && nv>0) cos_sim = dot / std::sqrt(nu*nv);
    }
    last_vecs[code] = v;

    if (std::isfinite(cos_sim)) {
        safe_publish<double>(TOP_PCA_STAB, code, ts, cos_sim);
        if (_cfg.debug_mode) {
            LOG_DEBUG("PCAStability[{}]: cos(Δθ)={:.6f}", code, cos_sim);
        }
    }
}

void PCAStructureStabilityFactor::on_quote(const QuoteDepth& q) {
    ensure_state(q.instrument_id);
    auto& s = _states[q.instrument_id];
    double mid = (q.bid_price + q.ask_price)/2.0;
    if (s.has_last_price && s.last_price>0.0) {
        double ret = std::log(mid / s.last_price);
        // 仅报价时没有 OFI/volume，用 0 代替
        feed_vec(q.instrument_id, q.data_time_ms, {ret, 0.0, s.ofi_acc});
        s.ofi_acc = 0.0; // 清零
    }
    s.last_price = mid; s.has_last_price = true;
}

void PCAStructureStabilityFactor::on_tick(const CombinedTick& x) {
    ensure_state(x.instrument_id);
    auto& s = _states[x.instrument_id];
    if (x.kind==CombinedKind::Trade) {
        // 累计 OFI（买=+，卖=-）
        double ofi = (x.side>0 ? 1.0 : -1.0) * static_cast<double>(x.volume);
        s.ofi_acc += ofi;
        if (s.has_last_price && s.last_price>0.0) {
            double ret = std::log(x.price / s.last_price);
            feed_vec(x.instrument_id, x.data_time_ms, {ret, static_cast<double>(x.volume), ofi});
            s.last_price = x.price; // 以成交近似
            return;
        }
        s.last_price = x.price;
        s.has_last_price = true;
    }
}

} // namespace factorlib
