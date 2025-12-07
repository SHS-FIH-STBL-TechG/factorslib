#include "factors/kline/volume_ar_forecast_factor.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/databus.h"
#include "utils/log.h"

namespace factorlib {

namespace {
constexpr const char* TOP_VOL_AR_RATIO = "kline/vol_ar1_pred_ratio"; // F10
constexpr double kEps = 1e-12;
}

VolumeArForecastFactor::CodeState::CodeState(const VolumeArForecastConfig& cfg)
    : ar1(static_cast<std::size_t>(std::max(cfg.ar_window, 2))),
      ma(static_cast<std::size_t>(std::max(cfg.ma_window, 1))),
      has_last_log(false),
      last_log_volume(0.0) {}

bool VolumeArForecastFactor::CodeState::push_bar(const Bar& b) {
    const double vol = static_cast<double>(b.volume);
    ma.push(vol);

    if (!(vol > 0.0)) {
        has_last_log = false;
        return false;
    }

    last_log_volume = std::log(vol);
    has_last_log = true;
    ar1.push(last_log_volume);
    return true;
}

VolumeArForecastFactor::VolumeArForecastFactor(
    const std::vector<Code>& codes,
    const VolumeArForecastConfig& cfg)
    : _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

bool VolumeArForecastFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void VolumeArForecastFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_VOL_AR_RATIO, capacity);
}

void VolumeArForecastFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg)).first;
    }
    CodeState& st = it->second;

    const bool has_log = st.push_bar(b);
    if (!has_log || !st.ready()) {
        return;
    }

    double alpha = 0.0;
    double phi = 0.0;
    double sigma2 = 0.0;
    if (!st.ar1.fit(alpha, phi, sigma2)) {
        return;
    }

    double last = st.last_log_volume;
    if (!std::isfinite(last)) {
        return;
    }

    const int horizon = std::max(_cfg.horizon, 1);
    double sum_pred = 0.0;
    for (int h = 0; h < horizon; ++h) {
        last = alpha + phi * last;
        sum_pred += std::exp(last);
    }
    const double avg_pred = sum_pred / static_cast<double>(horizon);

    const double base_vol = st.ma.mean();
    if (!(base_vol > 0.0) || !std::isfinite(base_vol)) {
        return;
    }

    const double value = std::log(avg_pred / (base_vol + kEps));
    if (!std::isfinite(value)) {
        return;
    }

    safe_publish<double>(TOP_VOL_AR_RATIO,
                         b.instrument_id,
                         b.data_time_ms,
                         value);
}

} // namespace factorlib
