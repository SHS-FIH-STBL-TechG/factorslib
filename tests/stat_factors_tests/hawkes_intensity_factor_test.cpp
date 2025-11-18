#include <gtest/gtest.h>
#include "../../src/stat_factors/hawkes_intensity_factor.h"
#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {
inline bool last_val(const std::string& code, double& v) {
    auto& bus = DataBus::instance(); int64_t ts=0;
    return bus.get_latest<double>(TOP_HAWKES, code, v, &ts);
}
}

TEST(HawkesIntensityFactor, BurstsIncreaseLambda) {
    HawkesIntensityFactor::register_topics(1024);
    std::string code="T1";
    HawkesCfg cfg; cfg.mu=0.1; cfg.alpha=0.5; cfg.beta=1.0; cfg.dt=1.0;
    HawkesIntensityFactor f({code}, cfg);

    // 两段爆发
    for (int t=0;t<50;++t) {
        CombinedTick x; x.instrument_id=code; x.data_time_ms=t; x.kind=CombinedKind::Trade; x.volume=1; x.price=100.0;
        f.on_tick(x);
    }
    double l1=0.0; ASSERT_TRUE(last_val(code,l1));
    for (int t=50;t<100;++t) {
        CombinedTick x; x.instrument_id=code; x.data_time_ms=t; x.kind=CombinedKind::Trade; x.volume=1; x.price=100.0;
        f.on_tick(x);
    }
    double l2=0.0; ASSERT_TRUE(last_val(code,l2));
    ASSERT_GT(l2, cfg.mu);
}
