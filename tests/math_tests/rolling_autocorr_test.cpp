#pragma once

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <limits>
#include <cstdlib>   // 为 std::rand / std::srand

#include "math/rolling_autocorr.h"
#include "math/bad_value_policy.h"

using namespace factorlib::math;

class RollingAutoCorrTest : public ::testing::Test {
protected:
    // 相关系数数值误差容忍度
    static constexpr double kCorrEps = 1e-12;

    void SetUp() override {
        // 通用测试配置（目前留空）
    }

    void TearDown() override {
        // 清理资源（目前留空）
    }

    // 辅助函数：生成线性序列
    std::vector<double> generate_linear_sequence(size_t n, double start = 0.0, double step = 1.0) {
        std::vector<double> seq;
        seq.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            seq.push_back(start + i * step);
        }
        return seq;
    }

    // 辅助函数：生成常数序列
    std::vector<double> generate_constant_sequence(size_t n, double value = 1.0) {
        return std::vector<double>(n, value);
    }

    // 辅助函数：生成正弦序列
    std::vector<double> generate_sine_sequence(size_t n, double amplitude = 1.0, double frequency = 1.0) {
        std::vector<double> seq;
        seq.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            seq.push_back(amplitude * std::sin(2 * M_PI * frequency * i / n));
        }
        return seq;
    }

    // 朴素实现：给定窗口数据和滞后 k，按定义计算自相关（与 RollingAutoCorr 的公式保持一致）
    static double naive_autocorr(const std::vector<double>& window, size_t k) {
        const size_t W = window.size();
        if (W == 0 || k >= W) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const size_t N = W - k;  // 样本对数量
        long double sum_x = 0.0L, sum_y = 0.0L;
        long double sum_xx = 0.0L, sum_yy = 0.0L, sum_xy = 0.0L;

        for (size_t i = 0; i < N; ++i) {
            long double x = static_cast<long double>(window[k + i]);
            long double y = static_cast<long double>(window[i]);
            sum_x  += x;
            sum_y  += y;
            sum_xx += x * x;
            sum_yy += y * y;
            sum_xy += x * y;
        }

        long double Nl = static_cast<long double>(N);
        long double mx = sum_x  / Nl;
        long double my = sum_y  / Nl;
        long double vx = sum_xx / Nl - mx * mx;
        long double vy = sum_yy / Nl - my * my;
        long double cov= sum_xy / Nl - mx * my;

        if (vx <= 0.0L || vy <= 0.0L) {
            return 0.0;  // 与 RollingAutoCorr 保持一致：方差不正时返回 0
        }

        long double r = cov / std::sqrt(vx * vy);
        return static_cast<double>(r);
    }
};

// ==================== 基础功能测试 ====================

/**
 * @test 测试基础构造和参数验证
 * @brief 验证构造函数正确处理窗口大小和滞后参数，检查边界条件
 */
TEST_F(RollingAutoCorrTest, ConstructorAndParameterValidation) {
    // 正常参数应该成功构造
    EXPECT_NO_THROW(RollingAutoCorr<double> rac(100, 5));

    // 滞后 k 必须小于窗口大小 W，否则抛出异常
    EXPECT_THROW(RollingAutoCorr<double> rac(10, 10), std::invalid_argument);
    EXPECT_THROW(RollingAutoCorr<double> rac(5, 6), std::invalid_argument);

    // 边界情况：k = W-1 应该成功
    EXPECT_NO_THROW(RollingAutoCorr<double> rac(100, 99));
}

/**
 * @test 测试窗口未满时的行为
 * @brief 验证在窗口填满之前，ready() 返回 false，value() 返回 NaN
 */
TEST_F(RollingAutoCorrTest, BehaviorBeforeWindowFull) {
    RollingAutoCorr<double> rac(10, 2);

    // 添加数据但未填满窗口
    for (int i = 0; i < 5; ++i) {
        EXPECT_TRUE(rac.push(i * 1.0));
        EXPECT_FALSE(rac.ready());
        EXPECT_TRUE(std::isnan(rac.value()));
    }

    // 窗口未满时 value 应该返回 NaN
    EXPECT_TRUE(std::isnan(rac.value()));
}

/**
 * @test 测试常数序列的自相关
 * @brief 验证常数序列的自相关系数为 0（因为方差为 0）
 */
TEST_F(RollingAutoCorrTest, ConstantSequenceAutoCorrelation) {
    RollingAutoCorr<double> rac(10, 1);
    auto constant_seq = generate_constant_sequence(15, 5.0);

    for (double val : constant_seq) {
        rac.push(val);
    }

    EXPECT_TRUE(rac.ready());
    // 常数序列方差为 0，根据实现应该返回 0.0
    EXPECT_DOUBLE_EQ(0.0, rac.value());
}

/**
 * @test 测试整型模板参数的支持
 * @brief 验证使用 int 类型时的数值行为与 double 朴素实现一致
 */
TEST_F(RollingAutoCorrTest, IntegerTypeAutoCorrelation) {
    const size_t W = 6, k = 2;
    RollingAutoCorr<int> rac(W, k);

    std::vector<int> data = {1, 2, 3, 4, 5, 6};
    for (int v : data) {
        rac.push(v);
    }

    EXPECT_TRUE(rac.ready());

    std::vector<double> window(data.begin(), data.end());
    double expected = naive_autocorr(window, k);
    double actual   = rac.value();

    EXPECT_NEAR(actual, expected, kCorrEps);
}

// ==================== 数值正确性测试 ====================

/**
 * @test 测试滞后 k=1 的线性序列自相关
 * @brief 验证线性递增序列在滞后 1 时有很强的正自相关，并与朴素实现一致
 */
TEST_F(RollingAutoCorrTest, LinearSequenceLag1AutoCorrelation) {
    const size_t W = 10;
    const size_t k = 1;
    RollingAutoCorr<double> rac(W, k);
    auto linear_seq = generate_linear_sequence(15, 0.0, 1.0);

    for (double val : linear_seq) {
        rac.push(val);
    }

    EXPECT_TRUE(rac.ready());

    // 当前窗口为最后 W 个样本
    std::vector<double> window(linear_seq.end() - W, linear_seq.end());
    double expected = naive_autocorr(window, k);
    double result   = rac.value();

    // 线性序列在滞后 1 时应该有很强的正自相关
    EXPECT_GT(result, 0.8);
    EXPECT_LE(result, 1.0 + kCorrEps);

    // 与朴素实现对齐
    EXPECT_NEAR(result, expected, kCorrEps);
}

/**
 * @test 测试滞后 k=2 的正弦序列自相关
 * @brief 验证周期性序列在适当滞后下具有明显自相关，并与朴素实现一致
 */
TEST_F(RollingAutoCorrTest, SineSequenceAutoCorrelation) {
    const size_t W = 20;
    const size_t k = 2;
    RollingAutoCorr<double> rac(W, k);
    auto sine_seq = generate_sine_sequence(25, 1.0, 0.1);

    for (double val : sine_seq) {
        rac.push(val);
    }

    EXPECT_TRUE(rac.ready());

    std::vector<double> window(sine_seq.end() - W, sine_seq.end());
    double expected = naive_autocorr(window, k);
    double result   = rac.value();

    // 正弦序列应该有明显的自相关
    EXPECT_GT(std::abs(result), 0.3);
    // 与朴素实现一致
    EXPECT_NEAR(result, expected, kCorrEps);
}

/**
 * @test 测试自相关结果范围
 * @brief 验证自相关值在 [-1, 1] 范围内，并与朴素实现匹配
 */
TEST_F(RollingAutoCorrTest, AutoCorrelationWithinRangeAndMatchesNaive) {
    const size_t W = 15, k = 3;
    RollingAutoCorr<double> rac(W, k);
    auto seq = generate_linear_sequence(20, 0.0, 0.5);

    for (double val : seq) {
        rac.push(val);
    }

    EXPECT_TRUE(rac.ready());

    std::vector<double> window(seq.end() - W, seq.end());
    double expected = naive_autocorr(window, k);
    double autocorr = rac.value();

    // 自相关应该在 [-1, 1] 范围内（考虑浮点误差）
    EXPECT_GE(autocorr, -1.0 - kCorrEps);
    EXPECT_LE(autocorr,  1.0 + kCorrEps);

    // 与朴素实现一致
    EXPECT_NEAR(autocorr, expected, kCorrEps);
}

// ==================== 滑动窗口测试 ====================

/**
 * @test 测试窗口滑动时的数值正确性
 * @brief 验证在连续 push 数据时，每一步计算结果与朴素 O(W) 实现一致
 */
TEST_F(RollingAutoCorrTest, SlidingWindowStability) {
    const size_t W = 8;
    const size_t k = 2;
    RollingAutoCorr<double> rac(W, k);

    // 较长线性序列，用于窗口滑动
    std::vector<double> long_sequence = generate_linear_sequence(30, 0.0, 1.0);

    for (size_t t = 0; t < long_sequence.size(); ++t) {
        rac.push(long_sequence[t]);

        if (t + 1 >= W) {
            EXPECT_TRUE(rac.ready());
            // 当前窗口对应最后 W 个样本
            std::vector<double> window(long_sequence.begin() + (t + 1 - W),
                                       long_sequence.begin() + (t + 1));
            double expected = naive_autocorr(window, k);
            double actual   = rac.value();

            EXPECT_NEAR(actual, expected, kCorrEps) << "滑动位置 t = " << t;
            EXPECT_GE(actual, -1.0 - kCorrEps);
            EXPECT_LE(actual,  1.0 + kCorrEps);
        } else {
            EXPECT_FALSE(rac.ready());
        }
    }
}

/**
 * @test 测试窗口完全滚动后的结果一致性
 * @brief 验证当窗口完全滚动替换后，计算结果与批处理朴素计算一致
 */
TEST_F(RollingAutoCorrTest, ConsistencyAfterFullRotation) {
    const size_t W = 6, k = 1;
    RollingAutoCorr<double> rac(W, k);

    // 第一轮数据
    std::vector<double> first_round = {1, 3, 5, 7, 9, 11};
    for (double val : first_round) {
        rac.push(val);
    }
    EXPECT_TRUE(rac.ready());
    double first_result = rac.value();
    double first_expected = naive_autocorr(first_round, k);

    // 第二轮数据（完全替换窗口）
    std::vector<double> second_round = {2, 4, 6, 8, 10, 12};
    for (double val : second_round) {
        rac.push(val);
    }
    double second_result = rac.value();
    double second_expected = naive_autocorr(second_round, k);

    // 两个结果都应为有效数值，且分别与朴素实现一致
    EXPECT_FALSE(std::isnan(first_result));
    EXPECT_FALSE(std::isnan(second_result));
    EXPECT_NEAR(first_result,  first_expected,  kCorrEps);
    EXPECT_NEAR(second_result, second_expected, kCorrEps);
}

// ==================== 边界条件测试 ====================

/**
 * @test 测试最小窗口情况
 * @brief 验证在最小可行窗口（W = k + 1）下的正确行为（只有一对样本）
 */
TEST_F(RollingAutoCorrTest, MinimumWindowSize) {
    // 最小窗口：W = k + 1 => 只有 1 个样本对
    RollingAutoCorr<double> rac(3, 2);  // W=3, k=2 → 只有 1 个 (X,Y)

    std::vector<double> seq = {1.0, 2.0, 3.0, 4.0};
    for (double val : seq) {
        rac.push(val);
    }

    EXPECT_TRUE(rac.ready());

    // 当前窗口为最后 3 个样本：{2,3,4}
    std::vector<double> window(seq.end() - 3, seq.end());
    double expected = naive_autocorr(window, 2);
    double result   = rac.value();

    // 只有一对样本时，方差为 0，约定返回 0
    EXPECT_DOUBLE_EQ(expected, 0.0);
    EXPECT_DOUBLE_EQ(result,   0.0);
}

/**
 * @test 测试不同滞后值的影响
 * @brief 验证不同滞后参数 k 对自相关结果的影响，并与朴素实现匹配
 */
TEST_F(RollingAutoCorrTest, DifferentLagValues) {
    const size_t W = 10;
    auto seq = generate_sine_sequence(15, 1.0, 0.2);

    std::vector<double> fast_results;
    std::vector<double> naive_results;

    for (size_t k = 1; k <= 3; ++k) {
        RollingAutoCorr<double> rac(W, k);
        for (double val : seq) {
            rac.push(val);
        }
        ASSERT_TRUE(rac.ready());

        std::vector<double> window(seq.end() - W, seq.end());
        double expected = naive_autocorr(window, k);
        double actual   = rac.value();

        fast_results.push_back(actual);
        naive_results.push_back(expected);

        EXPECT_NEAR(actual, expected, kCorrEps);
        EXPECT_GE(actual, -1.0 - kCorrEps);
        EXPECT_LE(actual,  1.0 + kCorrEps);
    }

    EXPECT_EQ(fast_results.size(), 3u);

    // 对于该固定的正弦序列，不同 k 通常会产生不同的自相关值
    // 这里简单检查至少有一对滞后产生的结果有明显差异
    EXPECT_NE(fast_results[0], fast_results[2]);
}

// ==================== 数值稳定性测试 ====================

/**
 * @test 测试极端数值情况
 * @brief 验证在极大值、极小值情况下的数值稳定性
 */
TEST_F(RollingAutoCorrTest, ExtremeNumericalValues) {
    RollingAutoCorr<double> rac(8, 2);

    // 混合极端值
    std::vector<double> extreme_seq = {
        1e-10, 1e10, -1e10, 1e-5,
        1e5, -1e5, 1e-8, 1e8,
        1e-12, 1e12, -1e12, 1e-3
    };

    for (double val : extreme_seq) {
        EXPECT_TRUE(rac.push(val));
    }

    if (rac.ready()) {
        double result = rac.value();
        // 结果应该在有效范围内（考虑浮点误差）
        EXPECT_GE(result, -1.0 - kCorrEps);
        EXPECT_LE(result,  1.0 + kCorrEps);
        EXPECT_FALSE(std::isnan(result));
    }
}

/**
 * @test 测试接近数值精度极限的情况
 * @brief 验证在浮点数精度边界时的稳健性
 */
TEST_F(RollingAutoCorrTest, NearPrecisionLimit) {
    RollingAutoCorr<double> rac(5, 1);

    // 使用接近机器精度的值
    double eps = std::numeric_limits<double>::epsilon();
    std::vector<double> precision_seq = {
        eps,
        2 * eps,
        3 * eps,
        4 * eps,
        5 * eps,
        6 * eps
    };

    for (double val : precision_seq) {
        EXPECT_TRUE(rac.push(val));
    }

    EXPECT_TRUE(rac.ready());
    double result = rac.value();
    // 即使是很小的值，也应该返回有效结果
    EXPECT_FALSE(std::isnan(result));
    EXPECT_GE(result, -1.0 - kCorrEps);
    EXPECT_LE(result,  1.0 + kCorrEps);
}

// ==================== 错误处理 / 坏值策略测试 ====================

/**
 * @test 测试 NoCheckBadValuePolicy 行为
 * @brief 验证在不检查坏值策略下，NaN 会传播导致最终结果为 NaN
 */
TEST_F(RollingAutoCorrTest, BadValueHandling_NoCheckBadValuePolicyPropagatesNaN) {
    RollingAutoCorr<double, NoCheckBadValuePolicy> rac(5, 1);

    EXPECT_TRUE(rac.push(1.0));
    EXPECT_TRUE(rac.push(2.0));
    EXPECT_TRUE(rac.push(3.0));
    EXPECT_TRUE(rac.push(4.0));

    // 推入 NaN，不做检查，直接参与计算
    double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(rac.push(nan));

    EXPECT_TRUE(rac.ready());
    double result = rac.value();
    EXPECT_TRUE(std::isnan(result));
}

/**
 * @test 测试 SkipNaNInfPolicy 行为
 * @brief 验证遇到 NaN / Inf 时，样本被丢弃，不参与窗口与统计量更新
 */
TEST_F(RollingAutoCorrTest, BadValueHandling_SkipNaNInfPolicySkipsSamples) {
    using RAC = RollingAutoCorr<double, SkipNaNInfPolicy>;
    const size_t W = 4, k = 1;
    RAC rac(W, k);

    // 第一个正常值
    EXPECT_TRUE(rac.push(1.0));
    EXPECT_FALSE(rac.ready());

    // NaN 应该被跳过（push 返回 false，窗口不增加）
    double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(rac.push(nan));
    EXPECT_FALSE(rac.ready());

    // 再推入 3 个正常值
    EXPECT_TRUE(rac.push(2.0));
    EXPECT_FALSE(rac.ready());
    EXPECT_TRUE(rac.push(3.0));
    EXPECT_FALSE(rac.ready());
    EXPECT_TRUE(rac.push(4.0));
    EXPECT_TRUE(rac.ready());

    // 预期窗口内容为 {1,2,3,4}
    std::vector<double> window = {1.0, 2.0, 3.0, 4.0};
    double expected = naive_autocorr(window, k);
    double actual   = rac.value();

    EXPECT_NEAR(actual, expected, kCorrEps);
}

/**
 * @test 测试 ZeroNaNInfPolicy 行为
 * @brief 验证遇到 NaN / Inf 时，会被替换为 0 再参与计算
 */
TEST_F(RollingAutoCorrTest, BadValueHandling_ZeroNaNInfPolicyReplacesWithZero) {
    using RAC = RollingAutoCorr<double, ZeroNaNInfPolicy>;
    const size_t W = 4, k = 1;
    RAC rac(W, k);

    double nan = std::numeric_limits<double>::quiet_NaN();
    double inf = std::numeric_limits<double>::infinity();

    EXPECT_TRUE(rac.push(1.0));
    EXPECT_TRUE(rac.push(nan)); // 被策略改为 0
    EXPECT_TRUE(rac.push(2.0));
    EXPECT_TRUE(rac.push(inf)); // 被策略改为 0

    EXPECT_TRUE(rac.ready());

    // 预期窗口内容为 {1,0,2,0}
    std::vector<double> window = {1.0, 0.0, 2.0, 0.0};
    double expected = naive_autocorr(window, k);
    double actual   = rac.value();

    EXPECT_NEAR(actual, expected, kCorrEps);
}

// ==================== 性能相关测试 ====================

/**
 * @test 测试大数据量下的内存使用
 * @brief 验证在长时间运行和大数据量下没有崩溃或异常，作为简单压力测试
 */
TEST_F(RollingAutoCorrTest, MemoryUsageWithLargeData) {
    const size_t W = 1000, k = 5;
    RollingAutoCorr<double> rac(W, k);

    // 推送大量数据（远大于窗口大小）
    for (int i = 0; i < 10000; ++i) {
        rac.push(static_cast<double>(i) + 0.1 * (i % 10));
    }

    EXPECT_TRUE(rac.ready());
    double result = rac.value();
    EXPECT_FALSE(std::isnan(result));
    EXPECT_GE(result, -1.0 - kCorrEps);
    EXPECT_LE(result,  1.0 + kCorrEps);

    // 实际内存泄漏检测需要配合工具，这里只做简单健壮性检查
}

/**
 * @test 测试连续操作的性能
 * @brief 验证增量实现的 O(1) 时间复杂度特性（至少不出现性能退化和异常）
 */
TEST_F(RollingAutoCorrTest, ContinuousOperationPerformance) {
    const size_t W = 50, k = 3;
    RollingAutoCorr<double> rac(W, k);

    const int NUM_OPERATIONS = 100000;
    for (int i = 0; i < NUM_OPERATIONS; ++i) {
        rac.push(static_cast<double>(i % 100));
        if (rac.ready()) {
            double val = rac.value();
            EXPECT_FALSE(std::isnan(val));
        }
    }

    // 测试能顺利跑完即可
    SUCCEED();
}

// ==================== 特殊序列测试 ====================

/**
 * @test 测试随机序列的自相关
 * @brief 验证随机序列的自相关通常应接近 0
 */
TEST_F(RollingAutoCorrTest, RandomSequenceAutoCorrelation) {
    RollingAutoCorr<double> rac(20, 1);

    // 使用伪随机序列（固定种子以便重现）
    std::srand(42);
    for (int i = 0; i < 25; ++i) {
        double random_val = static_cast<double>(std::rand()) / RAND_MAX;
        rac.push(random_val);
    }

    if (rac.ready()) {
        double result = rac.value();
        // 随机序列的自相关应该接近 0（设定一个宽松阈值）
        EXPECT_LT(std::abs(result), 0.5);
    }
}

/**
 * @test 测试交替序列的自相关
 * @brief 验证交替序列在滞后 1 时应该有强负自相关
 */
TEST_F(RollingAutoCorrTest, AlternatingSequenceAutoCorrelation) {
    RollingAutoCorr<double> rac(10, 1);

    // 创建交替序列：1, -1, 1, -1, ...
    for (int i = 0; i < 15; ++i) {
        rac.push((i % 2 == 0) ? 1.0 : -1.0);
    }

    EXPECT_TRUE(rac.ready());
    double result = rac.value();
    // 交替序列在滞后 1 时应该有强负自相关
    EXPECT_LT(result, -0.5);
    EXPECT_GE(result, -1.0 - kCorrEps);
}
