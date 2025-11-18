#include <gtest/gtest.h>
#include "../../src/stat_factors/volume_mgf_factor.h"
#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {
inline bool last_val(const std::string& code, double& v) {
    auto& bus = DataBus::instance(); int64_t ts=0;
    return bus.get_latest<double>(TOP_VOL_MGF, code, v, &ts);
}
}

TEST(VolumeMGFFactor, MonotoneInT) {
    VolumeMGFFactor::register_topics(1024);
    std::string code1="M1";
    VolMGFConfig cfg1; cfg1.window_size=64; cfg1.kmax=6; cfg1.t=0.01;
    VolumeMGFFactor f1({code1}, cfg1);
    std::string code2="M2";
    VolMGFConfig cfg2=cfg1; cfg2.t=0.05;
    VolumeMGFFactor f2({code2}, cfg2);

    for (int t=0;t<200;++t) {
        CombinedTick x; x.instrument_id=code1; x.data_time_ms=t; x.kind=CombinedKind::Trade; x.volume=100 + (t%10);
        f1.on_tick(x);
        x.instrument_id=code2; f2.on_tick(x);
    }
    double m1=0.0, m2=0.0; ASSERT_TRUE(last_val(code1,m1)); ASSERT_TRUE(last_val(code2,m2));
    ASSERT_GT(m2, m1) << "MGF 应随 t 增大而增大";
}
