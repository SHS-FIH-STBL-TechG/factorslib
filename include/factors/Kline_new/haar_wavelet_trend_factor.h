#pragma once
/**
 * @file haar_wavelet_trend_factor.h
 * @brief Haar 小波多分辨率趋势能量比因子。
 *
 * 理论依据：
 *   - 趋势是低频成分，噪声/波动更偏高频
 *   - 使用 Haar 小波进行多层分解
 *   - 输出低频趋势能量占比 × 方向符号
 *
 * 使用 math::RollingHaarWavelet 进行计算。
 */

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/databus.h"
#include "core/ifactor.h"
#include "core/types.h"
#include "math/rolling_haar_wavelet.h"

namespace factorlib {

/**
 * @brief Haar 小波趋势能量因子配置。
 */
struct HaarWaveletTrendConfig {
    int window_size = 32;   ///< 滚动窗口长度（需要是 2^levels 的倍数）
    int levels      = 2;    ///< 分解层数
};

/**
 * @brief Haar 小波多分辨率趋势能量比因子。
 *
 * 使用 math::RollingHaarWavelet 进行计算。
 */
class HaarWaveletTrendFactor : public BaseFactor {
public:
    using Code = std::string;

    HaarWaveletTrendFactor(
        const std::vector<Code>& codes,
        const HaarWaveletTrendConfig& cfg = HaarWaveletTrendConfig{}
    );

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;
    void on_quote(const QuoteDepth&) override {}
    void on_tick(const CombinedTick&) override {}

    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    struct CodeState {
        explicit CodeState(const HaarWaveletTrendConfig& cfg);

        math::RollingHaarWavelet<double> haar;

        void push_bar(const Bar& b);
        bool ready() const { return haar.ready(); }
    };

    HaarWaveletTrendConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
