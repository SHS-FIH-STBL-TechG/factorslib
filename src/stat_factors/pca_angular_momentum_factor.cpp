#include "pca_angular_momentum_factor.h"
#include <algorithm>

using factorlib::config::RC;

namespace factorlib {

PCAAngularMomentumFactor::PCAAngularMomentumFactor(const std::vector<std::string>& codes,
                                                   const PCAAngMomCfg& cfg)
    : BaseFactor("PCAAngularMomentum", codes), _cfg(cfg) {
    _cfg.dims = RC().geti("pca_ang.dims", _cfg.dims);
    _cfg.k    = RC().geti("pca_ang.k",    _cfg.k);
    _cfg.lr   = RC().getd("pca_ang.lr",   _cfg.lr);
    _cfg.debug_mode = RC().getb("pca_ang.debug_mode", _cfg.debug_mode);
}

void PCAAngularMomentumFactor::ensure_state(const std::string& code) {
    if (_states.find(code)!=_states.end()) return;
    CodeState st;
    st.pca = math::OnlinePCA(_cfg.dims, std::max(1,_cfg.k), _cfg.lr);
    st.last_mean.assign(_cfg.dims, 0.0);
    _states.emplace(code, std::move(st));
}

void PCAAngularMomentumFactor::feed_vec(const std::string& code, int64_t ts, const std::vector<double>& x) {
    (void)ts;
    ensure_state(code);
    auto& s = _states[code];
    s.pca.push(x);
    maybe_publish(code, ts);
}

static inline double norm3(const std::vector<double>& a){
    double s=0; for (size_t i=0;i<a.size();++i) s+=a[i]*a[i]; return std::sqrt(s);
}
static inline std::vector<double> cross3(const std::vector<double>& a, const std::vector<double>& b){
    std::vector<double> c(3,0.0);
    if (a.size()>=3 && b.size()>=3) {
        c[0] = a[1]*b[2] - a[2]*b[1];
        c[1] = a[2]*b[0] - a[0]*b[2];
        c[2] = a[0]*b[1] - a[1]*b[0];
    }
    return c;
}

void PCAAngularMomentumFactor::maybe_publish(const std::string& code, int64_t ts) {
    auto& s = _states[code];
    if (s.pca.n_samples() < 4) return;
    const auto& mean = s.pca.mean();
    std::vector<double> v(mean.size(), 0.0);
    for (size_t i=0;i<v.size();++i) v[i] = mean[i] - s.last_mean[i];
    s.last_mean = mean;
    // 第一主成分的“质量” ≈ 解释方差
    double m = 0.0;
    if (!s.pca.explained_variance().empty()) m = s.pca.explained_variance()[0];
    auto Lvec = cross3(mean, v);
    double L = m * norm3(Lvec);
    if (std::isfinite(L)) {
        safe_publish<double>(TOP_PCA_L, code, ts, L);
        if (_cfg.debug_mode) {
            LOG_DEBUG("PCAAngMom[{}]: m={:.4f}, |r×v|={:.4f}, L={:.6f}", code, m, norm3(Lvec), L);
        }
    }
}

void PCAAngularMomentumFactor::on_quote(const QuoteDepth& q) {
    ensure_state(q.instrument_id);
    auto& s = _states[q.instrument_id];
    double mid = (q.bid_price + q.ask_price)/2.0;
    if (s.has_last_price && s.last_price>0.0) {
        double ret = std::log(mid / s.last_price);
        feed_vec(q.instrument_id, q.data_time_ms, {ret, 0.0, 0.0});
    }
    s.last_price = mid; s.has_last_price = true;
}

void PCAAngularMomentumFactor::on_tick(const CombinedTick& x) {
    ensure_state(x.instrument_id);
    auto& s = _states[x.instrument_id];
    if (x.kind==CombinedKind::Trade) {
        double ofi = (x.side>0 ? 1.0 : -1.0) * static_cast<double>(x.volume);
        if (s.has_last_price && s.last_price>0.0) {
            double ret = std::log(x.price / s.last_price);
            feed_vec(x.instrument_id, x.data_time_ms, {ret, static_cast<double>(x.volume), ofi});
            s.last_price = x.price; 
            return;
        }
        s.last_price = x.price; s.has_last_price = true;
    }
}

} // namespace factorlib
