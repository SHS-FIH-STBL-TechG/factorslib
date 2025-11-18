#include "hawkes_intensity_factor.h"
#include <cmath>

using factorlib::config::RC;

namespace factorlib {

HawkesIntensityFactor::HawkesIntensityFactor(const std::vector<std::string>& codes,
                                             const HawkesCfg& cfg)
    : BaseFactor("HawkesIntensity", codes), _cfg(cfg) {
    _cfg.mu    = RC().getd("hawkes.mu",    _cfg.mu);
    _cfg.alpha = RC().getd("hawkes.alpha", _cfg.alpha);
    _cfg.beta  = RC().getd("hawkes.beta",  _cfg.beta);
    _cfg.dt    = RC().getd("hawkes.dt",    _cfg.dt);
    _cfg.debug_mode = RC().getb("hawkes.debug_mode", _cfg.debug_mode);
}

void HawkesIntensityFactor::ensure_state(const std::string& code) {
    auto it = _states.find(code);
    if (it==_states.end()) {
        CodeState st; st.init(_cfg);
        _states.emplace(code, std::move(st));
    }
}

void HawkesIntensityFactor::on_tick(const CombinedTick& x) {
    ensure_state(x.instrument_id);
    auto& st = _states[x.instrument_id];
    if (x.kind==CombinedKind::Trade) {
        double lambda = st.hawkes.update(1.0); // 事件=1
        safe_publish<double>(TOP_HAWKES, x.instrument_id, x.data_time_ms, lambda);
        if (_cfg.debug_mode) {
            LOG_DEBUG("Hawkes[{}]: λ={:.6f}", x.instrument_id, lambda);
        }
    } else {
        // 非成交：可做衰减更新（n_t=0）
        double lambda = st.hawkes.update(0.0);
        safe_publish<double>(TOP_HAWKES, x.instrument_id, x.data_time_ms, lambda);
    }
}

} // namespace factorlib
