#pragma once

#include <gtest/gtest.h>
#include <vector>
#include <deque>
#include <cmath>
#include <limits>
#include <random>
#include <numeric>

#include "math/statistics.h"

using namespace factorlib::math;

using StatsD = Statistics<double>;

// ==================== 统计工具测试 ====================

class StatisticsTest : public ::testing::Test {
protected:
    static constexpr double kTightTol = 1e-12;
    static constexpr double kLooseTol = 1e-6;

    // 辅助函数：手工计算均值
    double manual_mean(const std::vector<double>& v) {
        if (v.empty()) return 0.0;
        double s = 0.0;
        for (double x : v) s += x;
        return s / static_cast<double>(v.size());
    }

    // 辅助函数：手工计算样本标准差（除以 n-1）
    double manual_stddev(const std::vector<double>& v) {
        const std::size_t n = v.size();
        if (n < 2) return 0.0;
        double m = manual_mean(v);
        double sse = 0.0;
        for (double x : v) {
            double d = x - m;
            sse += d * d;
        }
        double var = sse / static_cast<double>(n - 1);
        return var > 0.0 ? std::sqrt(var) : 0.0;
    }

    // 辅助函数：手工计算样本协方差（除以 n-1）
    double manual_cov(const std::vector<double>& x, const std::vector<double>& y) {
        if (x.size() != y.size()) {
            // 理论上测试里不会传错，这里给个兜底
            return 0.0;
        }
        const std::size_t n = x.size();
        if (n < 2) return 0.0;
        double mx = manual_mean(x);
        double my = manual_mean(y);
        double s = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            s += (x[i] - mx) * (y[i] - my);
        }
        return s / static_cast<double>(n - 1);
    }

    // 辅助函数：滚动均值的朴素实现（窗口内跳过 NaN；全 NaN 时返回 0.0）
    std::vector<double> manual_rolling_mean_skip_nan(const std::vector<double>& data,
                                                     std::size_t window) {
        std::vector<double> result;
        const std::size_t N = data.size();
        if (window == 0 || N < window) return result;

        for (std::size_t i = 0; i + window <= N; ++i) {
            double s = 0.0;
            std::size_t cnt = 0;
            for (std::size_t j = 0; j < window; ++j) {
                double v = data[i + j];
                if (std::isnan(v)) continue;
                s += v;
                ++cnt;
            }
            if (cnt == 0) {
                result.push_back(0.0);
            } else {
                result.push_back(s / static_cast<double>(cnt));
            }
        }
        return result;
    }
};

// ==================== mean / stddev ====================

/**
 * @test 基础均值计算
 * @brief 验证无 NaN 情况下的均值与手工计算一致
 */
TEST_F(StatisticsTest, MeanBasicNoNaN) {
    std::vector<double> v = {1.0, 2.0, 3.0, 4.0};
    double mean = StatsD::mean(v);
    double ref  = manual_mean(v);
    EXPECT_NEAR(mean, ref, kTightTol);

    // 使用 deque 也应该相同
    std::deque<double> d(v.begin(), v.end());
    EXPECT_NEAR(StatsD::mean(d), ref, kTightTol);
}

/**
 * @test 空容器与全部 NaN 情况
 * @brief 验证 mean 对空容器和全 NaN 容器返回 0.0
 */
TEST_F(StatisticsTest, MeanOnEmptyAndAllNaN) {
    std::vector<double> empty;
    EXPECT_DOUBLE_EQ(StatsD::mean(empty), 0.0);

    std::vector<double> all_nan(5, std::numeric_limits<double>::quiet_NaN());
    EXPECT_DOUBLE_EQ(StatsD::mean(all_nan), 0.0);

    // 混合：NaN + 正常值，只计算正常值
    std::vector<double> mix = {
        std::numeric_limits<double>::quiet_NaN(),
        1.0,
        2.0,
        std::numeric_limits<double>::quiet_NaN()
    };
    EXPECT_NEAR(StatsD::mean(mix), (1.0 + 2.0) / 2.0, kTightTol);
}

/**
 * @test 标准差基础验证
 * @brief 对固定序列 [1,2,3,4] 验证样本标准差与手工计算一致
 */
TEST_F(StatisticsTest, StddevBasicSequence) {
    std::vector<double> v = {1.0, 2.0, 3.0, 4.0};  // mean=2.5, var=5/3
    double sd   = StatsD::stddev(v);
    double ref  = manual_stddev(v);  // sqrt(5/3)
    EXPECT_NEAR(sd, ref, kTightTol);
}

/**
 * @test 标准差的边界行为
 * @brief 样本数 < 2 或全部 NaN 时 stddev 返回 0.0
 */
TEST_F(StatisticsTest, StddevEdgeCases) {
    std::vector<double> empty;
    EXPECT_DOUBLE_EQ(StatsD::stddev(empty), 0.0);

    std::vector<double> single = {42.0};
    EXPECT_DOUBLE_EQ(StatsD::stddev(single), 0.0);

    std::vector<double> all_nan(3, std::numeric_limits<double>::quiet_NaN());
    EXPECT_DOUBLE_EQ(StatsD::stddev(all_nan), 0.0);

    std::vector<double> one_valid = {
        std::numeric_limits<double>::quiet_NaN(),
        5.0,
        std::numeric_limits<double>::quiet_NaN()
    };
    // 只有一个有效样本，也视作 n<2 -> 0
    EXPECT_DOUBLE_EQ(StatsD::stddev(one_valid), 0.0);
}

/**
 * @test 整型容器支持
 * @brief 验证对 std::vector<int> 也能正确计算均值和标准差
 */
TEST_F(StatisticsTest, MeanAndStddevForIntContainer) {
    std::vector<int> v = {1, 2, 3, 4, 5};
    std::vector<double> vd(v.begin(), v.end());

    EXPECT_NEAR(StatsD::mean(v),   manual_mean(vd),   kTightTol);
    EXPECT_NEAR(StatsD::stddev(v), manual_stddev(vd), kTightTol);
}

// ==================== quantile / median ====================

/**
 * @test 基本分位数（无 NaN）
 * @brief 验证 0 / 0.25 / 0.5 / 0.75 / 1 分位数的正确性
 */
TEST_F(StatisticsTest, QuantileBasic) {
    // 无序输入
    std::vector<double> v = {3.0, 1.0, 4.0, 2.0};

    // 排序后为 [1,2,3,4]
    EXPECT_NEAR(StatsD::quantile(v, 0.0), 1.0, kTightTol);   // 最小值
    EXPECT_NEAR(StatsD::quantile(v, 1.0), 4.0, kTightTol);   // 最大值

    // 中位数位置：percentile*(n-1)
    EXPECT_NEAR(StatsD::quantile(v, 0.5), 2.5, kTightTol);   // 介于2和3
    EXPECT_NEAR(StatsD::quantile(v, 0.25), 1.75, kTightTol); // 介于1和2
    EXPECT_NEAR(StatsD::quantile(v, 0.75), 3.25, kTightTol); // 介于3和4
}

/**
 * @test 分位数的 NaN 过滤
 * @brief 验证 quantile 会跳过 NaN，仅对有效样本排序
 */
TEST_F(StatisticsTest, QuantileWithNaN) {
    double NaN = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> v = {NaN, 1.0, NaN, 3.0, 2.0};

    // 有效数据为 [1,2,3]
    EXPECT_NEAR(StatsD::quantile(v, 0.0), 1.0, kTightTol);
    EXPECT_NEAR(StatsD::quantile(v, 0.5), 2.0, kTightTol);
    EXPECT_NEAR(StatsD::quantile(v, 1.0), 3.0, kTightTol);

    // 全部 NaN -> 0.0
    std::vector<double> all_nan(4, NaN);
    EXPECT_DOUBLE_EQ(StatsD::quantile(all_nan, 0.3), 0.0);
}

/**
 * @test 中位数封装
 * @brief median 应与 quantile(0.5) 行为一致
 */
TEST_F(StatisticsTest, MedianOddAndEven) {
    std::vector<double> odd  = {1.0, 3.0, 2.0};      // 排序 [1,2,3] -> 中位数 2
    std::vector<double> even = {4.0, 1.0, 3.0, 2.0}; // 排序 [1,2,3,4] -> 2.5

    EXPECT_NEAR(StatsD::median(odd),  2.0, kTightTol);
    EXPECT_NEAR(StatsD::median(even), 2.5, kTightTol);

    // 带 NaN
    double NaN = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> mix = {NaN, 3.0, 1.0, NaN, 2.0};
    EXPECT_NEAR(StatsD::median(mix), 2.0, kTightTol);
}

/**
 * @test 中位秩基本行为
 * @brief 验证 median_rank 的秩计算与 lower_bound 公式一致
 */
TEST_F(StatisticsTest, MedianRankBasic) {
    std::vector<double> v = {1.0, 2.0, 3.0, 4.0}; // 排序相同

    // value=1.0 -> rank=0 -> (0+0.5)/4 = 0.125
    EXPECT_NEAR(StatsD::median_rank(v, 1.0), 0.125, kTightTol);
    // value=2.5 -> lower_bound=2 -> (2+0.5)/4 = 0.625
    EXPECT_NEAR(StatsD::median_rank(v, 2.5), 0.625, kTightTol);
    // value=10 -> lower_bound=4 -> (4+0.5)/4 = 1.125
    EXPECT_NEAR(StatsD::median_rank(v, 10.0), 1.125, kTightTol);
}

/**
 * @test 中位秩边界情况
 * @brief 验证空容器 / value 为 NaN 时的退化行为
 */
TEST_F(StatisticsTest, MedianRankEdgeCases) {
    std::vector<double> empty;
    EXPECT_DOUBLE_EQ(StatsD::median_rank(empty, 123.0), 0.5);

    double NaN = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> v = {1.0, 2.0, 3.0};
    EXPECT_DOUBLE_EQ(StatsD::median_rank(v, NaN), 0.5);

    // 全部 NaN -> cleaned 为空 -> 0.5
    std::vector<double> all_nan(5, NaN);
    EXPECT_DOUBLE_EQ(StatsD::median_rank(all_nan, 1.0), 0.5);
}

// ==================== correlation / covariance ====================

/**
 * @test 完全正相关和完全负相关
 * @brief 验证 correlation 在简单线性关系下为 ±1
 */
TEST_F(StatisticsTest, CorrelationPerfectPositiveAndNegative) {
    // 完全正相关：y = 2x + 3
    std::vector<double> x = {1.0, 2.0, 3.0, 4.0};
    std::vector<double> y_pos;
    for (double v : x) y_pos.push_back(2.0 * v + 3.0);

    double r_pos = StatsD::correlation(x, y_pos);
    EXPECT_NEAR(r_pos, 1.0, 1e-12);
    EXPECT_LE(r_pos, 1.0 + 1e-9);

    // 完全负相关：y = -5x
    std::vector<double> y_neg;
    y_neg.reserve(x.size());
    for (double v : x) y_neg.push_back(-5.0 * v);

    double r_neg = StatsD::correlation(x, y_neg);
    EXPECT_NEAR(r_neg, -1.0, 1e-12);
    EXPECT_GE(r_neg, -1.0 - 1e-9);
}

/**
 * @test 带噪声的线性关系
 * @brief 对 y = a + b x + noise，相关系数应该接近 1
 */
TEST_F(StatisticsTest, CorrelationNoisyLinearRelationship) {
    std::mt19937_64 rng(20251126);
    std::normal_distribution<double> noise(0.0, 0.1);

    std::vector<double> x(200);
    std::vector<double> y(200);
    for (std::size_t i = 0; i < x.size(); ++i) {
        x[i] = static_cast<double>(i) / 10.0;
        y[i] = 1.0 + 2.0 * x[i] + noise(rng);
    }

    double r = StatsD::correlation(x, y);
    EXPECT_GT(r, 0.95);   // 相关性应非常强
    EXPECT_LE(r, 1.0 + 1e-9);
}

/**
 * @test 相关系数 NaN 过滤与边界行为
 * @brief 验证有 NaN 时 pair 被跳过，尺寸不匹配或有效样本不足时返回 0.0
 */
TEST_F(StatisticsTest, CorrelationNaNAndSizeMismatch) {
    double NaN = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> x = {1.0, NaN, 3.0, 4.0};
    std::vector<double> y = {2.0, 5.0, NaN, 8.0};

    // 有效 pair 只有 (1,2) 与 (4,8) -> 完全正相关
    double r = StatsD::correlation(x, y);
    EXPECT_NEAR(r, 1.0, 1e-12);

    // 尺寸不一致 -> 0.0
    std::vector<double> y_short = {1.0, 2.0};
    EXPECT_DOUBLE_EQ(StatsD::correlation(x, y_short), 0.0);

    // 全 NaN -> 0.0
    std::vector<double> all_nan(4, NaN);
    EXPECT_DOUBLE_EQ(StatsD::correlation(all_nan, all_nan), 0.0);
}

/**
 * @test 协方差与手工结果对比
 * @brief 对简单序列验证 covariance 与手工公式一致
 */
TEST_F(StatisticsTest, CovarianceBasic) {
    std::vector<double> x = {1.0, 2.0, 3.0, 4.0};
    std::vector<double> y = {2.0, 4.0, 6.0, 8.0}; // y=2x

    double cov  = StatsD::covariance(x, y);
    double ref  = manual_cov(x, y);

    EXPECT_NEAR(cov, ref, kTightTol);

    // 和相关系数的关系：r = cov / (sx * sy)
    double sx = StatsD::stddev(x);
    double sy = StatsD::stddev(y);
    double r  = StatsD::correlation(x, y);

    double r_ref = cov / (sx * sy);
    EXPECT_NEAR(r, r_ref, 1e-12);
}

/**
 * @test 协方差 NaN 过滤与边界行为
 * @brief 验证 covariance 对 NaN 对、尺寸不匹配等情况返回 0.0
 */
TEST_F(StatisticsTest, CovarianceNaNAndSizeMismatch) {
    double NaN = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> x = {1.0, NaN, 3.0};
    std::vector<double> y = {2.0, 4.0, NaN};

    // 有效 pair 只有 (1,2) -> n=1 -> 0.0
    EXPECT_DOUBLE_EQ(StatsD::covariance(x, y), 0.0);

    // 尺寸不匹配 -> 0.0
    std::vector<double> y_short = {2.0, 4.0};
    EXPECT_DOUBLE_EQ(StatsD::covariance(x, y_short), 0.0);
}

// ==================== rolling_mean ====================

/**
 * @test 滚动均值基础测试
 * @brief 验证 rolling_mean 在无 NaN 时与手工滑窗均值一致
 */
TEST_F(StatisticsTest, RollingMeanBasic) {
    std::vector<double> v = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::size_t W = 3;

    auto rm  = StatsD::rolling_mean(v, W);
    auto ref = manual_rolling_mean_skip_nan(v, W);

    ASSERT_EQ(rm.size(), ref.size());
    for (std::size_t i = 0; i < rm.size(); ++i) {
        EXPECT_NEAR(rm[i], ref[i], kTightTol);
    }
}

/**
 * @test 滚动均值中的 NaN 处理
 * @brief 验证窗口内 NaN 被过滤，全部 NaN 时结果为 0.0
 */
TEST_F(StatisticsTest, RollingMeanWithNaN) {
    double NaN = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> v = {1.0, NaN, 3.0, NaN, 5.0};
    std::size_t W = 2;

    auto rm  = StatsD::rolling_mean(v, W);
    auto ref = manual_rolling_mean_skip_nan(v, W);

    ASSERT_EQ(rm.size(), ref.size());
    for (std::size_t i = 0; i < rm.size(); ++i) {
        EXPECT_NEAR(rm[i], ref[i], kTightTol);
    }

    // 全部 NaN：每个窗口都应返回 0.0
    std::vector<double> all_nan = {NaN, NaN, NaN};
    auto rm2 = StatsD::rolling_mean(all_nan, 2);
    ASSERT_EQ(rm2.size(), 2u);
    EXPECT_DOUBLE_EQ(rm2[0], 0.0);
    EXPECT_DOUBLE_EQ(rm2[1], 0.0);
}

/**
 * @test 滚动均值窗口边界
 * @brief window_size 为0或大于数据长度时应返回空结果
 */
TEST_F(StatisticsTest, RollingMeanWindowEdgeCases) {
    std::vector<double> v = {1.0, 2.0, 3.0};

    auto r0 = StatsD::rolling_mean(v, 0);
    EXPECT_TRUE(r0.empty());

    auto r1 = StatsD::rolling_mean(v, 4);
    EXPECT_TRUE(r1.empty());
}

/**
 * @test 滚动均值长跑稳定性
 * @brief 在长序列上多次滑窗，检查结果有限且不会出现异常值
 */
TEST_F(StatisticsTest, RollingMeanLongRunStability) {
    std::mt19937_64 rng(20251126);
    std::normal_distribution<double> dist(0.0, 1.0);

    std::vector<double> v(1000);
    for (double& x : v) x = dist(rng);

    auto rm = StatsD::rolling_mean(v, 20);
    for (double m : rm) {
        EXPECT_TRUE(std::isfinite(m));
        // 正态(0,1) 的窗口均值大部分应在 [-5,5] 内，这里作为非常宽松的 sanity check
        EXPECT_LT(std::fabs(m), 10.0);
    }

    SUCCEED();
}
