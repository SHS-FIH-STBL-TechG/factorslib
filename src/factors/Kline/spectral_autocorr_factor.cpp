#include "factors/kline/spectral_autocorr_factor.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/databus.h"
#include "utils/log.h"

namespace factorlib {

namespace {
/**
 * @brief topic 名称：
 *   - 37. 低频能量占比        -> "kline/ret_spec_lowfreq_ratio"
 *   - 39. 频谱重心            -> "kline/ret_spec_centroid"
 *   - 40. 频谱斜率            -> "kline/ret_spec_slope"
 *   - 43. 自相关低阶能量      -> "kline/ret_ac_low_energy"
 *   - 44. 自相关高阶能量      -> "kline/ret_ac_high_energy"
 *   - 45. 自相关峰数量        -> "kline/ret_ac_peak_count"
 */
constexpr const char* TOP_LF_ENERGY_RATIO = "kline/ret_spec_lowfreq_ratio";
constexpr const char* TOP_SPEC_CENTROID   = "kline/ret_spec_centroid";
constexpr const char* TOP_SPEC_SLOPE      = "kline/ret_spec_slope";
constexpr const char* TOP_AC_LOW_ENERGY   = "kline/ret_ac_low_energy";
constexpr const char* TOP_AC_HIGH_ENERGY  = "kline/ret_ac_high_energy";
constexpr const char* TOP_AC_PEAK_COUNT   = "kline/ret_ac_peak_count";
} // anonymous namespace

//====================== CodeState 实现 ======================//

SpectralAutocorrFactor::CodeState::CodeState(const SpectralAutocorrConfig& cfg)
    : window(cfg.window_size),
      has_last_close(false),
      last_close(0.0),
      spectral(static_cast<std::size_t>(cfg.window_size)),
      acf(static_cast<std::size_t>(cfg.window_size),
          static_cast<std::size_t>(std::max(cfg.ac_low_max_lag, cfg.ac_high_max_lag))) {}

/**
 * @brief 推进一根 K 线，更新对数收益滑窗。
 */
bool SpectralAutocorrFactor::CodeState::push_bar(const Bar& b) {
    if (!(b.close > 0.0)) {
        return false;
    }

    const double close = static_cast<double>(b.close);
    if (!has_last_close) {
        last_close = close;
        has_last_close = true;
        return false;
    }

    const double ret = std::log(close / last_close);
    last_close = close;

    const bool spec_ready = spectral.push(ret);
    const bool acf_ready = acf.push(ret);
    return spec_ready && acf_ready;
}

int SpectralAutocorrFactor::CodeState::peak_count(std::size_t max_lag,
                                                  double threshold) const {
    if (!acf.ready() || max_lag <= 1) {
        return 0;
    }

    int count = 0;
    for (std::size_t lag = 1; lag + 1 <= max_lag; ++lag) {
        const double prev = (lag == 1) ? 1.0 : acf.acf(lag - 1);
        const double curr = acf.acf(lag);
        const double next = acf.acf(lag + 1);
        if (!std::isfinite(prev) || !std::isfinite(curr) || !std::isfinite(next)) {
            continue;
        }
        if (curr >= threshold && curr > prev && curr >= next) {
            ++count;
        }
    }
    return count;
}

//====================== 因子主体实现 ======================//

SpectralAutocorrFactor::SpectralAutocorrFactor(
    const std::vector<Code>& codes,
    const SpectralAutocorrConfig& cfg)
    : BaseFactor("SpectralAutocorrFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

/**
 * @brief 判断是否需要为指定代码计算本因子。
 */
bool SpectralAutocorrFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

/**
 * @brief 在 DataBus 上注册本因子输出的各个 topic。
 */
void SpectralAutocorrFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_LF_ENERGY_RATIO, capacity);
    bus.register_topic<double>(TOP_SPEC_CENTROID,   capacity);
    bus.register_topic<double>(TOP_SPEC_SLOPE,      capacity);
    bus.register_topic<double>(TOP_AC_LOW_ENERGY,   capacity);
    bus.register_topic<double>(TOP_AC_HIGH_ENERGY,  capacity);
    bus.register_topic<int64_t>(TOP_AC_PEAK_COUNT,  capacity);
}

/**
 * @brief 使用单根 K 线驱动频域 + 自相关联合因子计算。
 */
void SpectralAutocorrFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    // 找到 / 创建对应代码的滑窗状态
    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg)).first;
    }
    CodeState& st = it->second;

    // 更新对数收益滑窗
    if (!st.push_bar(b)) {
        return;
    }

    const int64_t ts = b.data_time_ms;

    //------------------ 频域特征 ------------------//
    // 37. 低频能量占比
    const double lf_ratio = st.spectral.low_freq_energy_ratio(
        static_cast<std::size_t>(_cfg.low_freq_bins));
    if (std::isfinite(lf_ratio)) {
        safe_publish<double>(TOP_LF_ENERGY_RATIO,
                             b.instrument_id,
                             ts,
                             lf_ratio);
    }

    // 39. 频谱重心
    const double centroid = st.spectral.spectral_centroid();
    if (std::isfinite(centroid)) {
        safe_publish<double>(TOP_SPEC_CENTROID,
                             b.instrument_id,
                             ts,
                             centroid);
    }

    // 40. 频谱斜率
    const double slope = st.spectral.spectral_slope();
    if (std::isfinite(slope)) {
        safe_publish<double>(TOP_SPEC_SLOPE,
                             b.instrument_id,
                             ts,
                             slope);
    }

    //------------------ 自相关特征 ------------------//
    const std::size_t ac_low_max_lag  = static_cast<std::size_t>(_cfg.ac_low_max_lag);
    const std::size_t ac_high_min_lag = static_cast<std::size_t>(_cfg.ac_high_min_lag);
    const std::size_t ac_high_max_lag = static_cast<std::size_t>(_cfg.ac_high_max_lag);

    // 43. 自相关低阶能量
    const double ac_low = st.acf.low_energy(ac_low_max_lag);
    if (std::isfinite(ac_low)) {
        safe_publish<double>(TOP_AC_LOW_ENERGY,
                             b.instrument_id,
                             ts,
                             ac_low);
    }

    // 44. 自相关高阶能量
    const double ac_high = st.acf.high_energy(ac_high_min_lag, ac_high_max_lag);
    if (std::isfinite(ac_high)) {
        safe_publish<double>(TOP_AC_HIGH_ENERGY,
                             b.instrument_id,
                             ts,
                             ac_high);
    }

    // 45. 自相关峰数量
    const int peak_cnt = st.peak_count(ac_high_max_lag, _cfg.ac_peak_threshold);
    safe_publish<int64_t>(TOP_AC_PEAK_COUNT,
                          b.instrument_id,
                          ts,
                          static_cast<int64_t>(peak_cnt));
}

} // namespace factorlib
