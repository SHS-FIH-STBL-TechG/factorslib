#include "factors/kline/low_freq_return_factor.h"

#include <algorithm>
#include <cmath>

namespace factorlib {

/*
（LowFreqReturnFactor）

- 输出 topic：`kline/ret_lowfreq_mu10`
- 含义：用频谱低频能量占比（慢变化成分强弱）乘以滑窗平均收益（方向），得到“趋势强度 × 方向”的组合指标。
- 典型场景：
  - 慢涨趋势：低频占比高且均值收益为正，输出显著为正。
  - 慢跌趋势：低频占比高但均值收益为负，输出显著为负。
  - 高频震荡：低频占比低，输出幅度趋小（即使均值收益略偏正/负）。
- 参数提示：`spectral_window` 控制频谱分辨率；`mean_window` 控制方向均值平滑程度；`low_freq_bins` 决定“低频”边界。
*/

namespace {
constexpr const char* TOP_LOW_FREQ_RET = "kline/ret_lowfreq_mu10"; // F7
}

LowFreqReturnFactor::CodeState::CodeState(const LowFreqReturnConfig& cfg)
    : has_last_close(false),
      last_close(0.0),
      spectral(static_cast<std::size_t>(std::max(cfg.spectral_window, 8))),
      mean_ret(static_cast<std::size_t>(std::max(cfg.mean_window, 2))) {}

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
    : BaseFactor("LowFreqReturnFactor", codes),
      _cfg(cfg),
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
