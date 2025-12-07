#include "factors/kline/low_freq_return_factor.h"

#include <algorithm>
#include <cmath>

#include "core/databus.h"
#include "utils/log.h"

namespace factorlib {

namespace {
constexpr const char* TOP_LOW_FREQ_RET = "kline/ret_lowfreq_mu10"; // F7
}

LowFreqReturnFactor::CodeState::CodeState(const LowFreqReturnConfig& cfg)
    : has_last_close(false),
      last_close(0.0),
      spectral(static_cast<std::size_t>(std::max(cfg.spectral_window, 8))),
      mean_ret(static_cast<std::size_t>(std::max(cfg.mean_window, 2)))) {}

bool LowFreqReturnFactor::CodeState::push_bar(const Bar& b) {
    if (!(b.close > 0.0)) return false;
    const double close = static_cast<double>(b.close);
    if (!has_last_close) {
        last_close = close;
        has_last_close = true;
        return false;
    }

    const double ret = std::log(close / last_close);
    last_close = close;
    if (!std::isfinite(ret)) {
        return false;
    }

    spectral.push(ret);
    mean_ret.push(ret);
    return ready();
}

LowFreqReturnFactor::LowFreqReturnFactor(
    const std::vector<Code>& codes,
    const LowFreqReturnConfig& cfg)
    : _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

bool LowFreqReturnFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void LowFreqReturnFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_LOW_FREQ_RET, capacity);
}

void LowFreqReturnFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg)).first;
    }
    CodeState& st = it->second;

    if (!st.push_bar(b) || !st.ready()) {
        return;
    }

    const double ratio = st.spectral.low_freq_energy_ratio(
        static_cast<std::size_t>(std::max(_cfg.low_freq_bins, 1)));
    const double mu = st.mean_ret.mean();
    if (!std::isfinite(ratio) || !std::isfinite(mu)) {
        return;
    }

    const double value = ratio * mu;
    safe_publish<double>(TOP_LOW_FREQ_RET,
                         b.instrument_id,
                         b.data_time_ms,
                         value);
}

} // namespace factorlib
