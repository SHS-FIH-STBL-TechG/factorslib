#include <gtest/gtest.h>
#include "../../src/stat_factors/pca_structure_stability_factor.h"
#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {
inline bool last_val(const std::string& code, double& v) {
    auto& bus = DataBus::instance(); int64_t ts=0;
    return bus.get_latest<double>(TOP_PCA_STAB, code, v, &ts);
}
}

TEST(PCAStructureStabilityFactor, StableDirectionCosineCloseToOne) {
    PCAStructureStabilityFactor::register_topics(1024);
    std::string code="PCA1";
    PCAStabilityCfg cfg; cfg.dims=3; cfg.k=1; cfg.lr=0.1;
    PCAStructureStabilityFactor f({code}, cfg);

    double p=100.0;
    for (int t=0;t<400;++t) {
        // 稳定方向：ret 随机波动很小，volume 与 ret 成正相关
        double ret = 0.001 + 0.0001*(t%5);
        p *= std::exp(ret);
        CombinedTick x; x.instrument_id=code; x.data_time_ms=t; x.kind=CombinedKind::Trade; x.price=p; x.volume=100 + int(ret*10000); x.side= (t%2==0?+1:-1);
        f.on_tick(x);
    }
    double s=0.0; ASSERT_TRUE(last_val(code,s));
    ASSERT_GT(s, 0.6);
}
