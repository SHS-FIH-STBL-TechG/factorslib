#pragma once

#include "core/ifactor.h"
#include "core/types.h"

#include "math/sliding_statistics.h"
#include "math/spectral_features.h"
#include "core/databus.h"
#include "utils/log.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

/**
 * @brief 配置：低频谱 × 10 日均收益（F7）。
 */
struct LowFreqReturnConfig {
    int spectral_window = 64;  ///< 频谱窗口长度
    int low_freq_bins = 3;     ///< 低频频点个数
    int mean_window = 10;      ///< 历史收益均值窗口
};

/**
 * @brief 因子：低频谱比率 × 历史均收益。
 *
 * - 先对对数收益序列做 RollingSpectralFeatures，得到低频能量占比；
 * - 再以 10 日滑窗计算历史对数收益均值；
 * - 因子值为二者乘积，衡量“慢趋势 × 持续收益”。
 */
class LowFreqReturnFactor : public BaseFactor {
public:
    using Code = std::string;

    LowFreqReturnFactor(
        const std::vector<Code>& codes,
        const LowFreqReturnConfig& cfg = LowFreqReturnConfig{}
    );

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;

    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    struct CodeState {
        explicit CodeState(const LowFreqReturnConfig& cfg);

        bool push_bar(const Bar& b);
        bool ready() const { return spectral.ready() && mean_ret.ready(); }

        bool has_last_close;
        double last_close;
        math::RollingSpectralFeatures<double> spectral;
        math::SlidingWindowStats<double> mean_ret;
    };

    LowFreqReturnConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
