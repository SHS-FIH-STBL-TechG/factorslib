#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "math/modwt.h"

using namespace factorlib::math;

// 测试目的：合成“趋势+高频”信号，验证 MODWT 趋势能量占比在合理区间（>0.5）
TEST(RollingMODWTTest, TrendEnergyRatioMatchesInternal) {
    std::size_t W=32; int J=3;
    auto wf = wavelet_db4();
    RollingMODWT<double> modwt(W, J, wf);

    std::vector<double> seq(W+16);
    for (size_t t=0;t<seq.size();++t) {
        seq[t] = 0.1*t + 0.5*std::sin(2*M_PI*t/8.0);
    }
    for (double x : seq) modwt.push(x);

    ASSERT_TRUE(modwt.ready());
    double r = modwt.trend_energy_ratio(2);
    EXPECT_GT(r, 0.5);
}
