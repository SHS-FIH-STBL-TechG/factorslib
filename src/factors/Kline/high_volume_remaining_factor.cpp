#include "factors/kline/high_volume_remaining_factor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace factorlib {

namespace {
constexpr const char* TOP_HIGH_VOL_REMAIN = "kline/vol_high_remaining"; // F14
}

HighVolumeRemainingFactor::CodeState::CodeState(const HighVolumeRemainingConfig& cfg)
    : quantile(static_cast<std::size_t>(std::max(cfg.volume_window, 1)), cfg.high_quantile),
      run_stats(static_cast<std::size_t>(std::max(cfg.run_history_window, 1)),
                static_cast<std::size_t>(std::max(cfg.max_run_length, 1))),
      last_threshold(std::numeric_limits<double>::quiet_NaN()),
      is_high(false) {}

bool HighVolumeRemainingFactor::CodeState::push_bar(const Bar& b) {
    const double vol = static_cast<double>(b.volume);
    quantile.push(vol);
    if (!quantile.ready()) {
        is_high = false;
        return false;
    }

    last_threshold = quantile.value();
    if (!std::isfinite(last_threshold)) {
        is_high = false;
        return false;
    }

    is_high = (vol > last_threshold);
    run_stats.push(is_high);
    return true;
}

HighVolumeRemainingFactor::HighVolumeRemainingFactor(
    const std::vector<Code>& codes,
    const HighVolumeRemainingConfig& cfg)
    : BaseFactor("HighVolumeRemainingFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

bool HighVolumeRemainingFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void HighVolumeRemainingFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_HIGH_VOL_REMAIN, capacity);
}

void HighVolumeRemainingFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg)).first;
    }
    CodeState& st = it->second;

    if (!st.push_bar(b) || !st.ready()) {
        return;
    }

    double value = 0.0;
    if (st.is_high) {
        const std::size_t age = st.run_stats.current_run_length();
        value = st.run_stats.expected_remaining(age);
    }

    safe_publish<double>(TOP_HIGH_VOL_REMAIN,
                         b.instrument_id,
                         b.data_time_ms,
                         value);
}

} // namespace factorlib
