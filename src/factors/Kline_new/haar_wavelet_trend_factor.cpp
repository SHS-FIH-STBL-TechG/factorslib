#include "factors/Kline_new/haar_wavelet_trend_factor.h"

#include <cmath>

namespace factorlib {

// topic 名称
constexpr const char* TOP_HAAR_TREND_ENERGY = "kline_new/haar_trend_energy";

//====================== CodeState 实现 ======================//

HaarWaveletTrendFactor::CodeState::CodeState(const HaarWaveletTrendConfig& cfg)
    : haar(static_cast<std::size_t>(cfg.window_size), cfg.levels) {}

void HaarWaveletTrendFactor::CodeState::push_bar(const Bar& b) {
    if (!(b.close > 0.0)) {
        return;
    }

    const double log_price = std::log(static_cast<double>(b.close));
    haar.push(log_price);
}

//====================== 因子主体实现 ======================//

HaarWaveletTrendFactor::HaarWaveletTrendFactor(
    const std::vector<Code>& codes,
    const HaarWaveletTrendConfig& cfg)
    : BaseFactor("HaarWaveletTrendFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

bool HaarWaveletTrendFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void HaarWaveletTrendFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_HAAR_TREND_ENERGY, capacity);
}

void HaarWaveletTrendFactor::on_bar(const Bar& b) {
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

    const double trend_energy = st.haar.trend_energy_ratio();

    if (!std::isfinite(trend_energy)) {
        return;
    }

    safe_publish<double>(TOP_HAAR_TREND_ENERGY,
                         b.instrument_id,
                         b.data_time_ms,
                         trend_energy);
}

} // namespace factorlib
