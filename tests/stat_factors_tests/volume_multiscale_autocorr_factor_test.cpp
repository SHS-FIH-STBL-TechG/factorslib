#include <gtest/gtest.h>
#include "../../src/stat_factors/volume_multiscale_autocorr_factor.h"
#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {
inline bool last_val(const std::string& code, double& v) {
    auto& bus = DataBus::instance(); int64_t ts=0;
    return bus.get_latest<double>(TOP_VOL_AC_MS, code, v, &ts);
}
}

TEST(VolumeMultiscaleAutocorrFactor, PositiveAutocorr) {
    VolumeMultiscaleAutocorrFactor::register_topics(1024);
    std::string code="V1";
    VolMultiACConfig cfg; cfg.window_size=64; cfg.lags={1,2,4,8};
    VolumeMultiscaleAutocorrFactor f({code}, cfg);

    // 自相关正的体量序列：缓慢上升 + 小噪声
    uint64_t base=100;
    for (int t=0;t<200;++t) {
        CombinedTick x; x.instrument_id=code; x.data_time_ms=t; x.kind=CombinedKind::Trade; x.volume=base + t/5;
        f.on_tick(x);
    }
    double r=0.0; ASSERT_TRUE(last_val(code,r));
    ASSERT_GT(r, 0.1);
}
