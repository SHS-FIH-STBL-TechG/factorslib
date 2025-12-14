#include "factors/Kline_new/ssa_trend_slope_factor.h"

#include <cmath>

namespace factorlib {

// topic 名称
constexpr const char* TOP_SSA_TREND_SLOPE  = "kline_new/ssa_trend_slope";
constexpr const char* TOP_SSA_ENERGY_RATIO = "kline_new/ssa_energy_ratio";

//====================== CodeState 实现 ======================//

SsaTrendSlopeFactor::CodeState::CodeState(const SsaTrendSlopeConfig& cfg)
    : ssa(static_cast<std::size_t>(cfg.window_size), cfg.L, cfg.r, cfg.slope_window) {}

void SsaTrendSlopeFactor::CodeState::push_bar(const Bar& b) {
    if (!(b.close > 0.0)) {
        return;
    }

    // 存储对数价格
    const double log_price = std::log(static_cast<double>(b.close));
    ssa.push(log_price);
}

//====================== 因子主体实现 ======================//

SsaTrendSlopeFactor::SsaTrendSlopeFactor(
    const std::vector<Code>& codes,
    const SsaTrendSlopeConfig& cfg)
    : BaseFactor("SsaTrendSlopeFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

bool SsaTrendSlopeFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void SsaTrendSlopeFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_SSA_TREND_SLOPE, capacity);
    bus.register_topic<double>(TOP_SSA_ENERGY_RATIO, capacity);
}

void SsaTrendSlopeFactor::on_bar(const Bar& b) {
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

    auto [slope, energy_ratio] = st.ssa.compute();

    if (!std::isfinite(slope) || !std::isfinite(energy_ratio)) {
        return;
    }

    safe_publish<double>(TOP_SSA_TREND_SLOPE,
                         b.instrument_id,
                         b.data_time_ms,
                         slope);

    safe_publish<double>(TOP_SSA_ENERGY_RATIO,
                         b.instrument_id,
                         b.data_time_ms,
                         energy_ratio);
}

} // namespace factorlib
