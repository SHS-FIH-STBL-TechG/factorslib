#pragma once

#include "core/ifactor.h"
#include "core/types.h"

#include "math/rolling_quantile.h"
#include "math/run_length_stats.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

struct HighVolumeRemainingConfig {
    int volume_window = 240;
    double high_quantile = 0.8;
    int run_history_window = 480;
    int max_run_length = 60;
};

/**
 * @brief 因子：高量状态剩余时间（F14，滑动窗口版）。
 */
class HighVolumeRemainingFactor : public BaseFactor {
public:
    using Code = std::string;

    HighVolumeRemainingFactor(
        const std::vector<Code>& codes,
        const HighVolumeRemainingConfig& cfg = HighVolumeRemainingConfig{}
    );

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;

    bool force_flush() override { return true; }

private:
    struct CodeState {
        explicit CodeState(const HighVolumeRemainingConfig& cfg);

        bool push_bar(const Bar& b);
        bool ready() const { return quantile.ready(); }
        double threshold() const { return quantile.value(); }

        math::RollingQuantile<double> quantile;
        math::RollingRunLengthStats run_stats;
        double last_threshold;
        bool is_high;
    };

    HighVolumeRemainingConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
