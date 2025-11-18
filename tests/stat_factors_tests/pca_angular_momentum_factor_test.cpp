#include <gtest/gtest.h>
#include "../../src/stat_factors/pca_angular_momentum_factor.h"
#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {
inline bool last_val(const std::string& code, double& v) {
    auto& bus = DataBus::instance(); int64_t ts=0;
    return bus.get_latest<double>(TOP_PCA_L, code, v, &ts);
}
}

TEST(PCAAngularMomentumFactor, ProducesFiniteL) {
    PCAAngularMomentumFactor::register_topics(1024);
    std::string code="PCA2";
    PCAAngMomCfg cfg; cfg.dims=3; cfg.k=1; cfg.lr=0.1;
    PCAAngularMomentumFactor f({code}, cfg);

    double p=100.0;
    for (int t=0;t<400;++t) {
        // 沿着圆弧移动的均值（通过周期性 ret/volume 逼近）
        double ret = 0.001*std::sin(0.05*t);
        p *= std::exp(ret);
        CombinedTick x; x.instrument_id=code; x.data_time_ms=t; x.kind=CombinedKind::Trade; x.price=p; x.volume=100 + int(10*std::cos(0.05*t)); x.side= (t%2==0?+1:-1);
        f.on_tick(x);
    }
    double L=0.0; ASSERT_TRUE(last_val(code,L));
    ASSERT_TRUE(std::isfinite(L));
}
