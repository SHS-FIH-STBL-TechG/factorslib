#pragma once

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include <random>

#include "math/sliding_statistics.h"
#include "math/bad_value_policy.h"

using namespace factorlib::math;

// ==================== SlidingWindowStats 测试 ====================

class SlidingWindowStatsTest : public ::testing::Test {
protected:
    static constexpr double kTightTol = 1e-12;
    static constexpr double kLooseTol = 1e-6;

    // 手工计算均值
    double manual_mean(const std::vector<double>& v) {
        if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
        double s = 0.0;
        for (double x : v) s += x;
        return s / static_cast<double>(v.size());
    }

    // 手工计算样本方差（除以 n-1）
    double manual_sample_var(const std::vector<double>& v) {
        const std::size_t n = v.size();
        if (n < 2) return std::numeric_limits<double>::quiet_NaN();
        double m = manual_mean(v);
        double sse = 0.0;
        for (double x : v) {
            double d = x - m;
            sse += d * d;
        }
        return sse / static_cast<double>(n - 1);
    }

    // 手工计算 SSE
    double manual_sse(const std::vector<double>& v) {
        const std::size_t n = v.size();
        if (n == 0) return 0.0;
        double m = manual_mean(v);
        double sse = 0.0;
        for (double x : v) {
            double d = x - m;
            sse += d * d;
        }
        return sse;
    }
};

/**
 * @test 基础构造与重置
 * @brief 验证默认构造、reset、clear 等基础行为
 */
TEST_F(SlidingWindowStatsTest, ConstructorResetAndClear) {
    // 默认构造：窗口大小为0，统计量为空
    SlidingWindowStats<double> s_default;
    EXPECT_EQ(s_default.window_size(), 0u);
    EXPECT_EQ(s_default.size(), 0u);
    EXPECT_FALSE(s_default.ready());
    EXPECT_FALSE(s_default.has_variance());
    EXPECT_TRUE(std::isnan(s_default.mean()));
    EXPECT_TRUE(std::isnan(s_default.variance_sample()));
    EXPECT_DOUBLE_EQ(s_default.sse(), 0.0);

    // 指定窗口大小构造
    SlidingWindowStats<double> stats(4);
    EXPECT_EQ(stats.window_size(), 4u);
    EXPECT_EQ(stats.size(), 0u);
    EXPECT_FALSE(stats.ready());

    // push 一些数据
    stats.push(1.0);
    stats.push(2.0);
    EXPECT_EQ(stats.size(), 2u);
    EXPECT_FALSE(stats.ready());

    // reset 更改窗口大小
    stats.reset(3);
    EXPECT_EQ(stats.window_size(), 3u);
    EXPECT_EQ(stats.size(), 0u);
    EXPECT_FALSE(stats.ready());
    EXPECT_TRUE(std::isnan(stats.mean()));

    // 再次填满并 clear()
    stats.push(1.0);
    stats.push(2.0);
    stats.push(3.0);
    EXPECT_EQ(stats.size(), 3u);
    EXPECT_TRUE(stats.ready());

    stats.clear();  // 等价于 reset(max_size_)
    EXPECT_EQ(stats.window_size(), 3u);
    EXPECT_EQ(stats.size(), 0u);
    EXPECT_FALSE(stats.ready());
}

/**
 * @test 简单序列的均值/方差/SSE
 * @brief 验证对固定窗口 [1,2,3,4] 的统计量与手工计算一致
 */
TEST_F(SlidingWindowStatsTest, SimpleSequenceStats) {
    SlidingWindowStats<double> stats(4);
    std::vector<double> v = {1.0, 2.0, 3.0, 4.0};
    for (double x : v) stats.push(x);

    ASSERT_EQ(stats.size(), 4u);
    ASSERT_TRUE(stats.ready());
    ASSERT_TRUE(stats.has_variance());

    double mean = stats.mean();
    double var  = stats.variance_sample();
    double sse  = stats.sse();

    double mean_ref = manual_mean(v);
    double var_ref  = manual_sample_var(v);
    double sse_ref  = manual_sse(v);

    EXPECT_NEAR(mean, mean_ref, kTightTol);
    EXPECT_NEAR(var,  var_ref,  kTightTol);
    EXPECT_NEAR(sse,  sse_ref,  kTightTol);
}

/**
 * @test 滑动窗口滚动行为
 * @brief 验证超出窗口后仅保留最近 max_size 个样本
 */
TEST_F(SlidingWindowStatsTest, SlidingWindowMovesCorrectly) {
    SlidingWindowStats<double> stats(3);

    // 推入 1,2,3
    stats.push(1.0);
    stats.push(2.0);
    stats.push(3.0);
    EXPECT_EQ(stats.size(), 3u);
    EXPECT_TRUE(stats.ready());

    // 此时窗口应为 [1,2,3]
    {
        std::vector<double> ref = {1.0, 2.0, 3.0};
        EXPECT_NEAR(stats.mean(), manual_mean(ref), kTightTol);
        EXPECT_NEAR(stats.variance_sample(), manual_sample_var(ref), kTightTol);
        EXPECT_NEAR(stats.sse(), manual_sse(ref), kTightTol);
    }

    // 再推入 4,5，最终窗口应为 [3,4,5]
    stats.push(4.0);
    stats.push(5.0);
    EXPECT_EQ(stats.size(), 3u);

    {
        std::vector<double> ref = {3.0, 4.0, 5.0};
        EXPECT_NEAR(stats.mean(), manual_mean(ref), kTightTol);
        EXPECT_NEAR(stats.variance_sample(), manual_sample_var(ref), kTightTol);
        EXPECT_NEAR(stats.sse(), manual_sse(ref), kTightTol);
    }
}

/**
 * @test 窗口大小为0时的行为
 * @brief max_size=0 被视为“无限窗口”：只增长不丢弃
 */
TEST_F(SlidingWindowStatsTest, ZeroWindowMeansInfiniteAccumulation) {
    SlidingWindowStats<double> stats;
    stats.reset(0);  // 显式设为0

    std::vector<double> v;
    for (int i = 1; i <= 100; ++i) {
        double x = static_cast<double>(i);
        v.push_back(x);
        stats.push(x);
    }

    EXPECT_EQ(stats.window_size(), 0u);
    EXPECT_EQ(stats.size(), v.size());
    EXPECT_FALSE(stats.ready());  // 永远不会 ready()

    double mean_ref = manual_mean(v);
    double var_ref  = manual_sample_var(v);
    double sse_ref  = manual_sse(v);

    EXPECT_NEAR(stats.mean(),            mean_ref, kLooseTol);
    EXPECT_NEAR(stats.variance_sample(), var_ref,  kLooseTol);
    EXPECT_NEAR(stats.sse(),            sse_ref,  kLooseTol);
}

/**
 * @test 样本不足时的方差行为
 * @brief 当窗口内样本数 < 2 时 variance_sample 应返回 NaN
 */
TEST_F(SlidingWindowStatsTest, VarianceRequiresAtLeastTwoSamples) {
    SlidingWindowStats<double> stats(5);
    EXPECT_FALSE(stats.has_variance());
    EXPECT_TRUE(std::isnan(stats.variance_sample()));

    stats.push(42.0);
    EXPECT_EQ(stats.size(), 1u);
    EXPECT_FALSE(stats.has_variance());
    EXPECT_TRUE(std::isnan(stats.variance_sample()));

    stats.push(43.0);
    EXPECT_EQ(stats.size(), 2u);
    EXPECT_TRUE(stats.has_variance());
    EXPECT_FALSE(std::isnan(stats.variance_sample()));
}

/**
 * @test SkipNaNInfPolicy 丢弃坏值
 * @brief 使用 SkipNaNInfPolicy 时，NaN/Inf 样本被丢弃，不进入窗口
 */
TEST_F(SlidingWindowStatsTest, SkipNaNInfPolicyDropsBadValues) {
    SlidingWindowStats<double, SkipNaNInfPolicy> stats(5);

    stats.push(1.0);
    stats.push(2.0);
    EXPECT_EQ(stats.size(), 2u);

    double mean_before = stats.mean();

    // 推入 NaN 和 Inf，应被策略丢弃
    double nan_v = std::numeric_limits<double>::quiet_NaN();
    double inf_v = std::numeric_limits<double>::infinity();

    stats.push(nan_v);
    stats.push(inf_v);
    EXPECT_EQ(stats.size(), 2u);  // 大小不变

    double mean_after = stats.mean();
    EXPECT_DOUBLE_EQ(mean_before, mean_after);

    // 再推入正常值
    stats.push(3.0);
    EXPECT_EQ(stats.size(), 3u);
    EXPECT_NEAR(stats.mean(), (1.0 + 2.0 + 3.0) / 3.0, kTightTol);
}

/**
 * @test ZeroNaNInfPolicy 将坏值替换为0
 * @brief 使用 ZeroNaNInfPolicy 时，NaN/Inf 被置为0后仍计入窗口
 */
TEST_F(SlidingWindowStatsTest, ZeroNaNInfPolicyReplacesBadValuesWithZero) {
    SlidingWindowStats<double, ZeroNaNInfPolicy> stats(5);

    double nan_v = std::numeric_limits<double>::quiet_NaN();
    double inf_v = std::numeric_limits<double>::infinity();

    stats.push(1.0);
    stats.push(nan_v);  // 将被替换为0
    stats.push(inf_v);  // 将被替换为0

    EXPECT_EQ(stats.size(), 3u);

    // 预期窗口为 [1, 0, 0]
    std::vector<double> ref = {1.0, 0.0, 0.0};
    EXPECT_NEAR(stats.mean(),            manual_mean(ref), kTightTol);
    EXPECT_NEAR(stats.variance_sample(), manual_sample_var(ref), kTightTol);
    EXPECT_NEAR(stats.sse(),            manual_sse(ref), kTightTol);
}

/**
 * @test 整型类型支持
 * @brief 验证 T=int 时统计量仍然正确（内部使用 long double 累积）
 */
TEST_F(SlidingWindowStatsTest, IntegerTypeSupport) {
    SlidingWindowStats<int> stats(4);
    std::vector<double> v = {1.0, 2.0, 3.0, 4.0};

    stats.push(1);
    stats.push(2);
    stats.push(3);
    stats.push(4);

    EXPECT_EQ(stats.size(), 4u);
    EXPECT_TRUE(stats.ready());

    double mean_ref = manual_mean(v);
    double var_ref  = manual_sample_var(v);
    double sse_ref  = manual_sse(v);

    EXPECT_NEAR(stats.mean(),            mean_ref, kTightTol);
    EXPECT_NEAR(stats.variance_sample(), var_ref,  kTightTol);
    EXPECT_NEAR(stats.sse(),            sse_ref,  kTightTol);
}

/**
 * @test 长时间运行的数值稳定性
 * @brief 对大量随机样本做滑动统计，验证均值/方差始终有限且方差在合理范围内
 */
TEST_F(SlidingWindowStatsTest, LongRunStability) {
    const std::size_t W = 50;
    SlidingWindowStats<double, SkipNaNInfPolicy> stats(W);

    std::mt19937_64 rng(20251126);
    std::normal_distribution<double> dist(0.0, 1.0);

    for (int i = 0; i < 5000; ++i) {
        double v = dist(rng);
        stats.push(v);

        if (stats.ready()) {
            double mu  = stats.mean();
            double var = stats.variance_sample();
            double sse = stats.sse();

            EXPECT_TRUE(std::isfinite(mu));
            EXPECT_TRUE(std::isfinite(var));
            EXPECT_TRUE(std::isfinite(sse));
            EXPECT_GE(var, 0.0);
            EXPECT_GE(sse, 0.0);

            // 正态(0,1) 的理论方差为1，这里只做非常宽松的 sanity check
            EXPECT_LT(var, 10.0);
        }
    }

    SUCCEED();
}

// ==================== WeightedSlidingWindowStats 测试 ====================

class WeightedSlidingWindowStatsTest : public ::testing::Test {
protected:
    static constexpr double kTightTol = 1e-12;
    static constexpr double kLooseTol = 1e-6;

    // 手工计算加权均值和方差（总体方差：除以 sum_w）
    void manual_weighted_stats(const std::vector<double>& vals,
                               const std::vector<double>& weights,
                               double& mu, double& sigma) {
        ASSERT_EQ(vals.size(), weights.size());
        const std::size_t n = vals.size();
        if (n == 0) {
            mu = 0.0; sigma = 0.0; return;
        }

        long double sw = 0.0L;
        long double m1 = 0.0L;
        long double m2 = 0.0L;
        for (std::size_t i = 0; i < n; ++i) {
            long double v = static_cast<long double>(vals[i]);
            long double w = static_cast<long double>(weights[i]);
            sw += w;
            m1 += w * v;
            m2 += w * v * v;
        }
        if (!(sw > 0.0L)) {
            mu = 0.0; sigma = 0.0; return;
        }
        m1 /= sw;
        m2 /= sw;
        long double var = m2 - m1 * m1;
        if (var < 0.0L) var = 0.0L;
        mu    = static_cast<double>(m1);
        sigma = std::sqrt(static_cast<double>(var));
    }
};

/**
 * @test 基本加权均值和方差
 * @brief 对固定样本 (1,2,3) 与权重 (1,2,3) 验证加权均值/方差
 */
TEST_F(WeightedSlidingWindowStatsTest, BasicWeightedMeanAndVariance) {
    WeightedSlidingWindowStats<double, double> ws(10);

    ws.push(1.0, 1.0);
    ws.push(2.0, 2.0);
    ws.push(3.0, 3.0);

    EXPECT_EQ(ws.size(), 3u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 6.0);

    double mu = 0.0, sigma = 0.0;
    ASSERT_TRUE(ws.mean_var(mu, sigma));

    // 手工结果：mean = 7/3, var = 5/9
    double mu_ref    = 7.0 / 3.0;
    double sigma_ref = std::sqrt(5.0 / 9.0);

    EXPECT_NEAR(mu,    mu_ref,    kTightTol);
    EXPECT_NEAR(sigma, sigma_ref, kTightTol);
}

/**
 * @test 窗口大小控制与滑动行为
 * @brief 验证加权窗口超长时，仅保留最近 max_size 条样本
 */
TEST_F(WeightedSlidingWindowStatsTest, SlidingWindowRespectsMaxSize) {
    WeightedSlidingWindowStats<double, double> ws(3);

    // 推入 (1,2,3,4)，权重全为1，最终窗口应为 (2,3,4)
    ws.push(1.0, 1.0);
    ws.push(2.0, 1.0);
    ws.push(3.0, 1.0);
    ws.push(4.0, 1.0);

    EXPECT_EQ(ws.size(), 3u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 3.0);

    double mu = 0.0, sigma = 0.0;
    ASSERT_TRUE(ws.mean_var(mu, sigma));

    std::vector<double> vals    = {2.0, 3.0, 4.0};
    std::vector<double> weights = {1.0, 1.0, 1.0};
    double mu_ref = 0.0, sigma_ref = 0.0;
    manual_weighted_stats(vals, weights, mu_ref, sigma_ref);

    EXPECT_NEAR(mu,    mu_ref,    kTightTol);
    EXPECT_NEAR(sigma, sigma_ref, kTightTol);
}

/**
 * @test 样本不足时 mean_var 行为
 * @brief size < 2 或 sum_w<=0 时 mean_var 返回 false，并将输出置为0
 */
TEST_F(WeightedSlidingWindowStatsTest, MeanVarNeedsAtLeastTwoSamples) {
    WeightedSlidingWindowStats<double, double> ws(5);

    double mu = -1.0, sigma = -1.0;
    EXPECT_FALSE(ws.mean_var(mu, sigma));
    EXPECT_DOUBLE_EQ(mu, 0.0);
    EXPECT_DOUBLE_EQ(sigma, 0.0);

    ws.push(1.0, 1.0);

    mu = -1.0; sigma = -1.0;
    EXPECT_FALSE(ws.mean_var(mu, sigma));
    EXPECT_DOUBLE_EQ(mu, 0.0);
    EXPECT_DOUBLE_EQ(sigma, 0.0);

    ws.push(2.0, 1.0);
    EXPECT_TRUE(ws.mean_var(mu, sigma));
}

/**
 * @test 非法权重被丢弃
 * @brief 权重为非有限值或 <=0 时，该样本不会被纳入统计
 */
TEST_F(WeightedSlidingWindowStatsTest, InvalidWeightsAreDropped) {
    WeightedSlidingWindowStats<double, double> ws(10);

    ws.push(1.0, 1.0);  // 有效
    EXPECT_EQ(ws.size(), 1u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 1.0);

    // 权重为0，丢弃
    ws.push(2.0, 0.0);
    EXPECT_EQ(ws.size(), 1u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 1.0);

    // 权重为负，丢弃
    ws.push(3.0, -1.0);
    EXPECT_EQ(ws.size(), 1u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 1.0);

    // 权重为 Inf，丢弃
    ws.push(4.0, std::numeric_limits<double>::infinity());
    EXPECT_EQ(ws.size(), 1u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 1.0);

    // 再加入两个合法样本
    ws.push(2.0, 2.0);
    ws.push(3.0, 3.0);
    EXPECT_EQ(ws.size(), 3u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 1.0 + 2.0 + 3.0);

    double mu = 0.0, sigma = 0.0;
    ASSERT_TRUE(ws.mean_var(mu, sigma));

    std::vector<double> vals    = {1.0, 2.0, 3.0};
    std::vector<double> weights = {1.0, 2.0, 3.0};
    double mu_ref = 0.0, sigma_ref = 0.0;
    manual_weighted_stats(vals, weights, mu_ref, sigma_ref);

    EXPECT_NEAR(mu,    mu_ref,    kTightTol);
    EXPECT_NEAR(sigma, sigma_ref, kTightTol);
}

/**
 * @test SkipNaNInfPolicy 在加权场景下丢弃坏 value
 * @brief value 为 NaN/Inf 时被策略丢弃，size 与 sum_w 不变
 */
TEST_F(WeightedSlidingWindowStatsTest, SkipNaNInfPolicyDropsBadValue) {
    WeightedSlidingWindowStats<double, double, SkipNaNInfPolicy> ws(10);

    ws.push(1.0, 1.0);
    EXPECT_EQ(ws.size(), 1u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 1.0);

    double nan_v = std::numeric_limits<double>::quiet_NaN();
    double inf_v = std::numeric_limits<double>::infinity();

    ws.push(nan_v, 2.0);  // 被丢弃
    ws.push(inf_v, 3.0);  // 被丢弃

    EXPECT_EQ(ws.size(), 1u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 1.0);

    double mu = 0.0, sigma = 0.0;
    EXPECT_FALSE(ws.mean_var(mu, sigma));  // 只有一个样本
}

/**
 * @test ZeroNaNInfPolicy 在加权场景下将坏 value 置0
 * @brief value 为 NaN/Inf 时被替换为0，但仍计入统计
 */
TEST_F(WeightedSlidingWindowStatsTest, ZeroNaNInfPolicyReplacesBadValueWithZero) {
    WeightedSlidingWindowStats<double, double, ZeroNaNInfPolicy> ws(10);

    double nan_v = std::numeric_limits<double>::quiet_NaN();
    double inf_v = std::numeric_limits<double>::infinity();

    ws.push(nan_v, 1.0);  // -> value=0
    ws.push(inf_v, 2.0);  // -> value=0
    ws.push(3.0,   3.0);  // 正常

    EXPECT_EQ(ws.size(), 3u);
    EXPECT_DOUBLE_EQ(ws.sum_w(), 6.0);

    double mu = 0.0, sigma = 0.0;
    ASSERT_TRUE(ws.mean_var(mu, sigma));

    // 等价于 vals=[0,0,3], w=[1,2,3]
    std::vector<double> vals    = {0.0, 0.0, 3.0};
    std::vector<double> weights = {1.0, 2.0, 3.0};
    double mu_ref = 0.0, sigma_ref = 0.0;
    manual_weighted_stats(vals, weights, mu_ref, sigma_ref);

    EXPECT_NEAR(mu,    mu_ref,    kTightTol);
    EXPECT_NEAR(sigma, sigma_ref, kTightTol);
}

/**
 * @test 与手工加权统计对比（随机数据）
 * @brief 对随机序列在滑窗上多次对比 mean_var 与手工结果
 */
TEST_F(WeightedSlidingWindowStatsTest, CompareWithManualRandomData) {
    const std::size_t W = 8;
    WeightedSlidingWindowStats<double, double> ws(W);

    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> val_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> w_dist(0.1, 2.0);

    std::vector<double> vals;
    std::vector<double> weights;

    for (int t = 0; t < 100; ++t) {
        double v = val_dist(rng);
        double w = w_dist(rng);

        ws.push(v, w);

        // 同步维护手工滑窗
        vals.push_back(v);
        weights.push_back(w);
        if (vals.size() > W) {
            vals.erase(vals.begin());
            weights.erase(weights.begin());
        }

        if (vals.size() >= 2) {
            double mu = 0.0, sigma = 0.0;
            ASSERT_TRUE(ws.mean_var(mu, sigma));

            double mu_ref = 0.0, sigma_ref = 0.0;
            manual_weighted_stats(vals, weights, mu_ref, sigma_ref);

            EXPECT_NEAR(mu,    mu_ref,    1e-10);
            EXPECT_NEAR(sigma, sigma_ref, 1e-10);
        }
    }
}

/**
 * @test 长时间运行稳定性
 * @brief 大量 push 随机样本，检查 mean_var 不产生 NaN/Inf 且权重和非负
 */
TEST_F(WeightedSlidingWindowStatsTest, LongRunStability) {
    const std::size_t W = 20;
    WeightedSlidingWindowStats<double, double> ws(W);

    std::mt19937_64 rng(98765);
    std::normal_distribution<double> val_dist(0.0, 1.0);
    std::uniform_real_distribution<double> w_dist(0.1, 1.0);

    for (int i = 0; i < 2000; ++i) {
        double v = val_dist(rng);
        double w = w_dist(rng);
        ws.push(v, w);

        if (ws.size() >= 2) {
            double mu = 0.0, sigma = 0.0;
            ASSERT_TRUE(ws.mean_var(mu, sigma));

            EXPECT_TRUE(std::isfinite(mu));
            EXPECT_TRUE(std::isfinite(sigma));
            EXPECT_GE(ws.sum_w(), 0.0);

            // 非常宽松范围检查
            EXPECT_LT(std::fabs(mu),    10.0);
            EXPECT_LT(sigma,            10.0);
        }
    }

    SUCCEED();
}
