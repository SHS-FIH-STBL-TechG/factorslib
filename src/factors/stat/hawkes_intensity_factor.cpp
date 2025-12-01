#include "factors/stat/hawkes_intensity_factor.h"
#include <cmath>
#include "config/config_utils.h"

using factorlib::config::RC;

namespace factorlib {

HawkesIntensityFactor::HawkesIntensityFactor(const std::vector<std::string>& codes,
                                             const HawkesCfg& cfg)
    : BaseFactor("HawkesIntensity", codes), _cfg(cfg) {
    _cfg.mu    = RC().getd("hawkes.mu",    _cfg.mu);
    _cfg.alpha = RC().getd("hawkes.alpha", _cfg.alpha);
    _cfg.beta  = RC().getd("hawkes.beta",  _cfg.beta);
    _cfg.dt    = RC().getd("hawkes.dt",    _cfg.dt);
    _window_sizes = {0};
}

HawkesIntensityFactor::CodeState& HawkesIntensityFactor::ensure_state(const ScopeKey& scope) {
    auto key = scope.as_bus_code();
    auto it = _states.find(key);
    if (it==_states.end()) {
        CodeState st; st.init(_cfg);
        it = _states.emplace(std::move(key), std::move(st)).first;
    }
    return it->second;
}

void HawkesIntensityFactor::on_tick(const CombinedTick& x) {
    ensure_code(x.instrument_id);
    for_each_scope(x.instrument_id, _window_sizes, x.data_time_ms, [&](const ScopeKey& scope) {
        auto& st = ensure_state(scope);

        if (x.data_time_ms < st.last_ts) {
            return; // 忽略过时数据
        }

        const std::string scoped_code = scope.as_bus_code();
        double lambda = 0.0;
        if (x.kind==CombinedKind::Trade) {
            lambda = st.hawkes.update(1.0); // 事件=1
            LOG_DEBUG("Hawkes[{}]: λ={:.6f}", scoped_code, lambda);
        } else {
            // 非成交：可做衰减更新（n_t=0）
            lambda = st.hawkes.update(0.0);
        }
        st.last_ts = x.data_time_ms;
        safe_publish<double>(TOP_HAWKES, scoped_code, x.data_time_ms, lambda);
    });
}

} // namespace factorlib
