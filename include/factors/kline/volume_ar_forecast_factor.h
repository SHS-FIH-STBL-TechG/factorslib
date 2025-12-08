#pragma once

#include "core/ifactor.h"
#include "core/types.h"

#include "math/rolling_ar.h"
#include "math/sliding_statistics.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

struct VolumeArForecastConfig {
    int ar_window = 120;
    int ma_window = 20;
    int horizon = 5;
};

/**
 * @brief 因子：成交量 AR(1) 多步预测相对 20 日均量的 log 比（F10）。
 */
class VolumeArForecastFactor : public BaseFactor {
public:
    using Code = std::string;

    VolumeArForecastFactor(
        const std::vector<Code>& codes,
        const VolumeArForecastConfig& cfg = VolumeArForecastConfig{}
    );

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;

    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    struct CodeState {
        explicit CodeState(const VolumeArForecastConfig& cfg);

        bool push_bar(const Bar& b);
        bool ready() const { return ar1.ready() && ma.ready() && has_last_log; }

        math::RollingAR1<double> ar1;
        math::SlidingWindowStats<double> ma;
        bool has_last_log;
        double last_log_volume;
    };

    VolumeArForecastConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
