// tests/math_tests/sliding_statistics_test.cpp
#include <vector>
#include <cmath>

#include <gtest/gtest.h>

#include "math/sliding_statistics.h"

using factorlib::math::SlidingWindowStats;
using factorlib::math::WeightedSlidingWindowStats;

// ===================== SlidingWindowStats 原有行为 =====================

TEST(SlidingWindowStatsTest, BasicWindowMeanVarSSE) {
    SlidingWindowStats<double> stats(3);

    // 先推 3 个样本：[1,2,3]
    stats.push(1.0);
    stats.push(2.0);
    stats.push(3.0);

    EXPECT_TRUE(stats.ready());
    EXPECT_EQ(stats.size(), 3u);

    // [1,2,3] mean=2, sample var=1, SSE=2
    EXPECT_NEAR(stats.mean(), 2.0, 1e-12);
    EXPECT_NEAR(stats.variance_sample(), 1.0, 1e-12);
    EXPECT_NEAR(stats.sse(), 2.0, 1e-12);

    // 再 push 一个 4，窗口应该变成 [2,3,4]
    stats.push(4.0);
    EXPECT_TRUE(stats.ready());
    EXPECT_EQ(stats.size(), 3u);

    // [2,3,4] mean=3, sample var=1, SSE=2
    EXPECT_NEAR(stats.mean(), 3.0, 1e-12);
    EXPECT_NEAR(stats.variance_sample(), 1.0, 1e-12);
    EXPECT_NEAR(stats.sse(), 2.0, 1e-12);
}

TEST(SlidingWindowStatsTest, ClearAndRefill) {
    SlidingWindowStats<double> stats(3);

    // 先填满
    stats.push(1.0);
    stats.push(2.0);
    stats.push(3.0);

    // 然后 clear
    stats.clear();
    EXPECT_EQ(stats.size(), 0u);
    EXPECT_FALSE(stats.ready());

    // 再重新填：[10,20,30]
    stats.push(10.0);
    stats.push(20.0);
    stats.push(30.0);

    EXPECT_TRUE(stats.ready());
    EXPECT_NEAR(stats.mean(), 20.0, 1e-12);
}

// ===================== WeightedSlidingWindowStats 新增行为 =====================

TEST(WeightedSlidingWindowStatsTest, EmptyWindowHasNoStats) {
    WeightedSlidingWindowStats<double, double> stats(3);

    double mu = 123.0;
    double sigma = 456.0;
    EXPECT_FALSE(stats.mean_var(mu, sigma));  // 样本不足
    EXPECT_EQ(stats.size(), 0u);
    EXPECT_DOUBLE_EQ(stats.sum_w(), 0.0);
}

TEST(WeightedSlidingWindowStatsTest, EqualWeightsMatchesManualComputation) {
    // r = [0.01, 0.02, -0.01], w = [1, 1, 1]
    // 期望:
    //   mu    = 0.006666666666666665
    //   sigma ≈ 0.012472191289246473
    WeightedSlidingWindowStats<double, double> stats(10);

    stats.push(0.01, 1.0);
    stats.push(0.02, 1.0);
    stats.push(-0.01, 1.0);

    EXPECT_EQ(stats.size(), 3u);
    EXPECT_DOUBLE_EQ(stats.sum_w(), 3.0);

    double mu = 0.0;
    double sigma = 0.0;
    ASSERT_TRUE(stats.mean_var(mu, sigma));

    const double mu_expected    = 0.006666666666666665;
    const double sigma_expected = 0.012472191289246473;

    EXPECT_NEAR(mu,    mu_expected,    1e-15);
    EXPECT_NEAR(sigma, sigma_expected, 1e-15);
}

TEST(WeightedSlidingWindowStatsTest, WeightedCaseMatchesManualComputation) {
    // r = [0.01, 0.02, -0.01], w = [1, 2, 3]
    // 期望:
    //   mu    ≈ 0.003333333333333334
    //   sigma ≈ 0.013743685418725535
    WeightedSlidingWindowStats<double, double> stats(10);

    stats.push(0.01, 1.0);
    stats.push(0.02, 2.0);
    stats.push(-0.01, 3.0);

    EXPECT_EQ(stats.size(), 3u);
    EXPECT_DOUBLE_EQ(stats.sum_w(), 6.0);

    double mu = 0.0;
    double sigma = 0.0;
    ASSERT_TRUE(stats.mean_var(mu, sigma));

    const double mu_expected    = 0.003333333333333334;
    const double sigma_expected = 0.013743685418725535;

    EXPECT_NEAR(mu,    mu_expected,    1e-15);
    EXPECT_NEAR(sigma, sigma_expected, 1e-15);
}

TEST(WeightedSlidingWindowStatsTest, RespectsWindowSizeSliding) {
    // 窗口大小 3，但推 5 个样本，前两个应该被逐出
    WeightedSlidingWindowStats<double, double> stats(3);

    stats.push(1.0, 1.0);  // 将被淘汰
    stats.push(2.0, 1.0);  // 将被淘汰
    stats.push(3.0, 1.0);
    stats.push(4.0, 1.0);
    stats.push(5.0, 1.0);

    EXPECT_EQ(stats.size(), 3u);         // 只保留最后三条
    EXPECT_DOUBLE_EQ(stats.sum_w(), 3.0);

    double mu = 0.0;
    double sigma = 0.0;
    ASSERT_TRUE(stats.mean_var(mu, sigma));

    // 最后三个值：[3,4,5]，等权 -> mean=4, var=2/3, std≈0.8164965809
    const double mu_expected    = 4.0;
    const double sigma_expected = 0.816496580927726; // sqrt(2/3)

    EXPECT_NEAR(mu,    mu_expected,    1e-12);
    EXPECT_NEAR(sigma, sigma_expected, 1e-12);
}
