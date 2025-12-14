#include "factors/Kline_new/dmd_growth_rate_factor.h"

#include <cmath>

namespace factorlib {

// topic 名称
constexpr const char* TOP_DMD_GROWTH_RATE = "kline_new/dmd_growth_rate";

//====================== CodeState 实现 ======================//

DmdGrowthRateFactor::CodeState::CodeState(const DmdGrowthRateConfig& cfg)
    : dmd(static_cast<std::size_t>(cfg.window_size), cfg.embed_dim, cfg.rank) {}

void DmdGrowthRateFactor::CodeState::push_bar(const Bar& b) {
    if (!(b.close > 0.0)) {
        return;
    }

    const double log_price = std::log(static_cast<double>(b.close));
    dmd.push(log_price);
}

//====================== 因子主体实现 ======================//

DmdGrowthRateFactor::DmdGrowthRateFactor(
    const std::vector<Code>& codes,
    const DmdGrowthRateConfig& cfg)
    : BaseFactor("DmdGrowthRateFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

bool DmdGrowthRateFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void DmdGrowthRateFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_DMD_GROWTH_RATE, capacity);
}

void DmdGrowthRateFactor::on_bar(const Bar& b) {
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

    const double growth_rate = st.dmd.compute();

    if (!std::isfinite(growth_rate)) {
        return;
    }

    safe_publish<double>(TOP_DMD_GROWTH_RATE,
                         b.instrument_id,
                         b.data_time_ms,
                         growth_rate);
}

} // namespace factorlib
