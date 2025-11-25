// tests/stat_factors_tests/volume_multiscale_autocorr_factor_test.cpp

#include "../../src/stat_factors/volume_multiscale_autocorr_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {

inline int64_t ms_of(int h, int m, int s, int ms = 0) {
    return (((static_cast<int64_t>(h) * 60 + m) * 60) + s) * 1000 + ms;
}

    struct BusGuard {
        BusGuard()  { DataBus::instance().reset(); }
        ~BusGuard() { DataBus::instance().reset(); }
    };

    bool last_vol_multi_ac(const std::string& code, double& out, int64_t* ts = nullptr) {
        return DataBus::instance().get_latest<double>(TOP_VOL_MULTI_AC, code, out, ts);
    }

    std::vector<Bar> make_bars(const std::string& code, size_t count) {
        std::vector<Bar> bars;
        bars.reserve(count);
        double px = 80.0;
        int64_t base_ts = ms_of(9, 45, 0, 0);
        for (size_t i = 0; i < count; ++i) {
            double ret = 0.0004 * std::sin(0.3 * static_cast<double>(i));
            px *= std::exp(ret);
            Bar b{};
            b.instrument_id = code;
            b.data_time_ms  = base_ts + static_cast<int64_t>(i) * 60'000;
            b.close         = px;
            b.volume        = 2000 + static_cast<int64_t>(i) * ((i % 5) + 1) * 30;
            bars.push_back(b);
        }
        return bars;
    }

} // namespace

// 基本用例：从合成的 Bar 序列检查多尺度自相关因子是否产出有限值
TEST(VolumeMultiscaleAutocorrFactor, SyntheticBarsProduceFiniteAutocorr) {
    BusGuard guard;
    VolumeMultiscaleAutocorrFactor::register_topics(2048);

    const std::string code = "VOL_MULTI_SYN";
    auto bars = make_bars(code, 160);

    // 这里直接用默认配置，不再显式写 VolMultiACConfig 那种不存在的类型
    VolumeMultiscaleAutocorrFactor factor({code});

    for (const auto& b : bars) {
        if (b.instrument_id != code) continue;
        factor.on_bar(b);
    }

    double v  = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_vol_multi_ac(code, v, &ts))
        << "Bars CSV 喂完后，应至少产出一次 volume/autocorr_multiscale";

    EXPECT_TRUE(std::isfinite(v));
    EXPECT_GE(v, -1.0);
    EXPECT_LE(v,  1.0);
}
