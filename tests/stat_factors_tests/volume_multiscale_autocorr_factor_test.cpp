// tests/stat_factors_tests/volume_multiscale_autocorr_factor_test.cpp

#include "../../src/stat_factors/volume_multiscale_autocorr_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "utils/databus.h"
#include "utils/types.h"
#include "../utils/test_config.h"   // testcfg::read_bars_from_cfg()

using namespace factorlib;

namespace {

    struct BusGuard {
        BusGuard()  { DataBus::instance().reset(); }
        ~BusGuard() { DataBus::instance().reset(); }
    };

    bool last_vol_multi_ac(const std::string& code, double& out, int64_t* ts = nullptr) {
        return DataBus::instance().get_latest<double>(TOP_VOL_MULTI_AC, code, out, ts);
    }

} // namespace

// 基本用例：从 bars_csv 回放成交量，检查多尺度自相关因子是否产出有限值
TEST(VolumeMultiscaleAutocorrFactor, BarsCsvFeed_Basic) {
    BusGuard guard;
    VolumeMultiscaleAutocorrFactor::register_topics(2048);

    auto bars = testcfg::read_bars_from_cfg();
    ASSERT_FALSE(bars.empty()) << "bars_csv 未配置或无有效数据";

    const std::string code = bars[0].instrument_id;

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
