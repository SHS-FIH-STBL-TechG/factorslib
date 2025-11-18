#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "math/hawkes_intensity.h"

using namespace factorlib::math;

// 测试用例 1：验证 Hawkes 指数核递推公式与手算一致（前两步严格比较）
// 入参：μ=0.1, α=0.3, β=1.2, Δt=1.0，事件序列 n_t = [1,0,1,0]
// 期望：手动使用 λ_{t+1} = μ + e^{-βΔt} (λ_t-μ) + α n_t 递推，前两步结果与实现完全一致。
TEST(HawkesIntensityTest, RecurrenceMatchesManual) {
    HawkesIntensity H(0.1, 0.3, 1.2, 1.0);
    std::vector<int> n{1,0,1,0};
    std::vector<double> lambda;
    for (int e : n) lambda.push_back(H.update(e));
    double exp1 = 0.1 + std::exp(-1.2)*(0.1-0.1) + 0.3*1.0;
    double exp2 = 0.1 + std::exp(-1.2)*(lambda[0]-0.1) + 0.3*0.0;
    EXPECT_NEAR(lambda[0], exp1, 1e-12);
    EXPECT_NEAR(lambda[1], exp2, 1e-12);
}

// 测试用例 2：验证单次脉冲后强度向基线 μ 指数衰减
// 入参：同样 μ=0.1, α=0.3, β=1.2, Δt=1.0，事件序列 n_t = [1,0,0,0,0]
// 期望：从 λ_0=μ 出发，显式递推 5 步得到强度序列，并与实现对比：
//   λ1 = 0.4
//   λ2 ≈ 0.19035826357366065
//   λ3 ≈ 0.12721538598682378
//   λ4 ≈ 0.10819711673418778
//   λ5 ≈ 0.10246892411470601
// 这些值应当逐步向 μ=0.1 收敛。
TEST(HawkesIntensityTest, SingleImpulseDecaysToBaseline) {
    HawkesIntensity H(0.1, 0.3, 1.2, 1.0);
    std::vector<int> n{1,0,0,0,0};
    std::vector<double> lambda;
    for (int e : n) lambda.push_back(H.update(e));

    ASSERT_EQ(lambda.size(), 5u);
    EXPECT_NEAR(lambda[0], 0.40000000000000002, 1e-12);
    EXPECT_NEAR(lambda[1], 0.19035826357366065, 1e-12);
    EXPECT_NEAR(lambda[2], 0.12721538598682378, 1e-12);
    EXPECT_NEAR(lambda[3], 0.10819711673418778, 1e-12);
    EXPECT_NEAR(lambda[4], 0.10246892411470601, 1e-12);

    // 额外检查：后续值应当单调地向基线 μ 接近
    EXPECT_GT(lambda[0], lambda[1]);
    EXPECT_GT(lambda[1], lambda[2]);
    EXPECT_GT(lambda[2], lambda[3]);
    EXPECT_GT(lambda[3], lambda[4]);
    EXPECT_GT(lambda[4], 0.1);
}
