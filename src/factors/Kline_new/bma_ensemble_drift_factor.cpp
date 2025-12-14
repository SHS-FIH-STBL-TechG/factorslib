#include "factors/Kline_new/bma_ensemble_drift_factor.h"

#include <cmath>

namespace factorlib {

// topic 名称
constexpr const char* TOP_BMA_ENSEMBLE_DRIFT = "kline_new/bma_ensemble_drift";

//====================== CodeState 实现 ======================//

BmaEnsembleDriftFactor::CodeState::CodeState(const BmaEnsembleDriftConfig& cfg)
    : has_last_close(false),
      last_close(0.0),
      bma(static_cast<std::size_t>(cfg.window_size), cfg.ema_alpha) {}

void BmaEnsembleDriftFactor::CodeState::push_bar(const Bar& b) {
    if (!(b.close > 0.0)) {
        return;
    }

    const double close = static_cast<double>(b.close);
    if (!has_last_close) {
        last_close = close;
        has_last_close = true;
        return;
    }

    // 对数收益
    const double ret = std::log(close / last_close);
    last_close = close;

    bma.push(ret);
}

//====================== 因子主体实现 ======================//

BmaEnsembleDriftFactor::BmaEnsembleDriftFactor(
    const std::vector<Code>& codes,
    const BmaEnsembleDriftConfig& cfg)
    : BaseFactor("BmaEnsembleDriftFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

bool BmaEnsembleDriftFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void BmaEnsembleDriftFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_BMA_ENSEMBLE_DRIFT, capacity);
}

void BmaEnsembleDriftFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg)).first;
    }
    CodeState& st = it->second;

    st.push_bar(b);
    if (!st.ready()) {
        return;
    }

    const double drift = st.bma.compute();

    if (!std::isfinite(drift)) {
        return;
    }

    safe_publish<double>(TOP_BMA_ENSEMBLE_DRIFT,
                         b.instrument_id,
                         b.data_time_ms,
                         drift);
}

} // namespace factorlib
