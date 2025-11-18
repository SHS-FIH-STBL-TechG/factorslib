#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "math/info_geometry.h"

using namespace factorlib::math;

// 测试用例 1：Bernoulli(θ) 的 Fisher 信息显式公式
// 入参：θ = 0.3
// 期望：I(θ) = 1 / [θ (1-θ)] ≈ 4.761904762
TEST(InfoGeometryTest, FisherBernoulliInterior) {
    double theta = 0.3;
    double expected = 1.0 / (theta * (1.0 - theta));  // ≈ 4.761904762
    double got = fisher_bernoulli(theta);
    EXPECT_NEAR(got, expected, 1e-12);
}

// 测试用例 2：Bernoulli 在边界 θ=0 或 1 时 Fisher 信息发散
// 入参：θ = 0, 1
// 期望：返回值为 +∞
TEST(InfoGeometryTest, FisherBernoulliBoundaryInfinity) {
    double t0 = fisher_bernoulli(0.0);
    double t1 = fisher_bernoulli(1.0);
    EXPECT_TRUE(std::isinf(t0));
    EXPECT_TRUE(std::isinf(t1));
}

// 测试用例 3：多项式分布 Fisher 度量的对角近似 I_ii ≈ 1 / p_i
// 入参：p = [0.2, 0.3, 0.5]
// 期望：g_i = 1/p_i = [5.0, 3.3333333333, 2.0]
// 说明：由于十进制字面量 3.3333333333 和 1/0.3 的二进制表示略有差异，
//       这里采用 1e-10 的容忍度即可覆盖这类舍入误差。
TEST(InfoGeometryTest, FisherMultinomialDiag) {
    std::vector<double> p{0.2, 0.3, 0.5};
    auto g = fisher_multinomial_diag(p);
    ASSERT_EQ(g.size(), p.size());
    EXPECT_NEAR(g[0], 5.0000000000, 1e-12);
    EXPECT_NEAR(g[1], 3.3333333333, 1e-10);
    EXPECT_NEAR(g[2], 2.0000000000, 1e-12);
}
