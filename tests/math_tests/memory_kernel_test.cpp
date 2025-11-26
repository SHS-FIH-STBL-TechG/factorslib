#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <random>
#include "math/memory_kernel.h"

using namespace factorlib::math;

/**
 * @brief 记忆核估计器测试夹具类
 * @brief 提供常用的测试数据和验证方法
 */
class MemoryKernelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 每个测试前的设置
    }

    void TearDown() override {
        // 每个测试后的清理
    }

    /**
     * @brief 生成常数序列
     * @param value 常数值
     * @param length 序列长度
     * @return 常数序列向量
     */
    std::vector<double> generate_constant_sequence(double value, size_t length) {
        return std::vector<double>(length, value);
    }

    /**
     * @brief 生成白噪声序列
     * @param mean 均值
     * @param stddev 标准差
     * @param length 序列长度
     * @param seed 随机种子
     * @return 白噪声序列向量
     */
    std::vector<double> generate_white_noise(double mean, double stddev,
                                           size_t length, unsigned seed = 42) {
        std::vector<double> sequence(length);
        std::default_random_engine generator(seed);
        std::normal_distribution<double> distribution(mean, stddev);

        for (size_t i = 0; i < length; ++i) {
            sequence[i] = distribution(generator);
        }
        return sequence;
    }

    /**
     * @brief 生成自回归AR(1)序列
     * @param phi 自回归系数
     * @param noise_std 噪声标准差
     * @param length 序列长度
     * @param seed 随机种子
     * @return AR(1)序列向量
     */
    std::vector<double> generate_ar1_sequence(double phi, double noise_std,
                                            size_t length, unsigned seed = 42) {
        std::vector<double> sequence(length);
        std::default_random_engine generator(seed);
        std::normal_distribution<double> distribution(0, noise_std);

        sequence[0] = distribution(generator);
        for (size_t i = 1; i < length; ++i) {
            sequence[i] = phi * sequence[i-1] + distribution(generator);
        }
        return sequence;
    }

    /**
     * @brief 验证记忆核值是否在合理范围内
     * @param value 记忆核值
     * @param min_val 最小值边界
     * @param max_val 最大值边界
     */
    void expect_valid_memory_kernel(double value, double min_val = -2.0, double max_val = 2.0) {
        EXPECT_FALSE(std::isnan(value)) << "Memory kernel should not be NaN";
        EXPECT_FALSE(std::isinf(value)) << "Memory kernel should not be infinite";
        EXPECT_GE(value, min_val) << "Memory kernel should be >= " << min_val;
        EXPECT_LE(value, max_val) << "Memory kernel should be <= " << max_val;
    }
};

/**
 * @brief 测试常数序列的记忆核估计
 * @brief 验证对于常数序列，记忆核应该接近于零
 */
TEST_F(MemoryKernelTest, ConstantSequenceReturnsZero) {
    const size_t W = 100;
    const size_t L = 10;
    const double alpha = 0.5;

    MemoryKernelEstimator<double> estimator(W, L, alpha);
    auto constant_seq = generate_constant_sequence(5.0, W + 10);

    for (double value : constant_seq) {
        EXPECT_TRUE(estimator.push(value));
    }

    EXPECT_TRUE(estimator.ready());
    double memory_kernel = estimator.value();
    EXPECT_NEAR(memory_kernel, 0.0, 1e-10) << "Memory kernel should be zero for constant sequence";
}

/**
 * @brief 测试白噪声序列的记忆核估计
 * @brief 验证对于白噪声，记忆核应该接近于零（无记忆性）
 */
TEST_F(MemoryKernelTest, WhiteNoiseHasSmallMemoryKernel) {
    const size_t W = 200;
    const size_t L = 20;
    const double alpha = 0.3;

    MemoryKernelEstimator<double> estimator(W, L, alpha);
    auto white_noise = generate_white_noise(0.0, 1.0, W + 50);

    for (double value : white_noise) {
        estimator.push(value);
    }

    EXPECT_TRUE(estimator.ready());
    double memory_kernel = estimator.value();
    EXPECT_NEAR(memory_kernel, 0.0, 0.1) << "White noise should have small memory kernel";
    expect_valid_memory_kernel(memory_kernel);
}

/**
 * @brief 测试强自相关序列的记忆核估计
 * @brief 验证对于强自相关序列，记忆核应该显著不为零
 */
TEST_F(MemoryKernelTest, StrongAutocorrelationHasSignificantMemoryKernel) {
    const size_t W = 150;
    const size_t L = 15;
    const double alpha = 0.7;

    MemoryKernelEstimator<double> estimator(W, L, alpha);
    auto ar_sequence = generate_ar1_sequence(0.9, 0.1, W + 30);

    for (double value : ar_sequence) {
        estimator.push(value);
    }

    EXPECT_TRUE(estimator.ready());
    double memory_kernel = estimator.value();
    EXPECT_GT(std::abs(memory_kernel), 0.1) << "Strong autocorrelation should have significant memory kernel";
    expect_valid_memory_kernel(memory_kernel);
}

/**
 * @brief 测试数据不足时的行为
 * @brief 验证在数据不足时，估计器返回NaN且ready()返回false
 */
TEST_F(MemoryKernelTest, ReturnsNaNWhenNotEnoughData) {
    const size_t W = 100;
    const size_t L = 10;
    const double alpha = 0.5;

    MemoryKernelEstimator<double> estimator(W, L, alpha);
    auto short_sequence = generate_white_noise(0.0, 1.0, W - 10);

    for (double value : short_sequence) {
        estimator.push(value);
    }

    EXPECT_FALSE(estimator.ready());
    double memory_kernel = estimator.value();
    EXPECT_TRUE(std::isnan(memory_kernel)) << "Should return NaN when not enough data";
}

/**
 * @brief 测试不同alpha参数的影响
 * @brief 验证alpha参数确实影响记忆核的权重分配
 */
TEST_F(MemoryKernelTest, AlphaParameterAffectsWeights) {
    const size_t W = 100;
    const size_t L = 10;

    auto sequence = generate_ar1_sequence(0.8, 0.2, W + 10);
    std::vector<double> alphas = {0.1, 0.5, 0.9};
    std::vector<double> memory_kernels;

    for (double alpha : alphas) {
        MemoryKernelEstimator<double> est(W, L, alpha);
        for (double value : sequence) {
            est.push(value);
        }
        memory_kernels.push_back(est.value());
    }

    // 验证不同alpha产生不同的记忆核值
    for (size_t i = 1; i < memory_kernels.size(); ++i) {
        EXPECT_NE(memory_kernels[i], memory_kernels[i-1])
            << "Different alpha values should produce different memory kernels";
    }
}

/**
 * @brief 测试边界条件处理
 * @brief 验证在边界条件下的鲁棒性
 */
TEST_F(MemoryKernelTest, BoundaryConditions) {
    // 测试最小有效配置
    EXPECT_NO_THROW({
        MemoryKernelEstimator<double> estimator1(10, 5, 0.5);
    });

    // 测试无效配置（L+1 >= W）
    EXPECT_THROW({
        MemoryKernelEstimator<double> estimator2(10, 10, 0.5);
    }, std::invalid_argument);

    // 测试极端alpha值
    EXPECT_NO_THROW({
        MemoryKernelEstimator<double> estimator3(100, 10, 0.001);
    });

    EXPECT_NO_THROW({
        MemoryKernelEstimator<double> estimator4(100, 10, 0.999);
    });
}

/**
 * @brief 测试数值稳定性
 * @brief 验证在极端数值输入下的稳定性
 */
TEST_F(MemoryKernelTest, NumericalStability) {
    const size_t W = 100;
    const size_t L = 10;
    const double alpha = 0.5;

    MemoryKernelEstimator<double> estimator(W, L, alpha);

    // 测试包含极端值的序列
    std::vector<double> extreme_sequence = {
        1e10, -1e10, 1e-10, -1e-10, 0.0,
        std::numeric_limits<double>::epsilon(),
        -std::numeric_limits<double>::epsilon()
    };

    // 重复极端序列直到填满窗口
    for (size_t i = 0; i < W * 2; ++i) {
        double value = extreme_sequence[i % extreme_sequence.size()];
        estimator.push(value);
    }

    EXPECT_TRUE(estimator.ready());
    double memory_kernel = estimator.value();

    EXPECT_FALSE(std::isnan(memory_kernel));
    EXPECT_FALSE(std::isinf(memory_kernel));
}

/**
 * @brief 测试配置参数获取方法
 * @brief 验证window_size()和max_lag()方法正确返回配置参数
 */
TEST_F(MemoryKernelTest, ConfigurationGetters) {
    const size_t W = 100;
    const size_t L = 10;
    const double alpha = 0.5;

    MemoryKernelEstimator<double> estimator(W, L, alpha);

    EXPECT_EQ(estimator.window_size(), W);
    EXPECT_EQ(estimator.max_lag(), L);
}

/**
 * @brief 测试坏值处理策略 - NoCheckBadValuePolicy
 * @brief 验证不检查策略会传播NaN，导致记忆核为NaN
 */
TEST_F(MemoryKernelTest, NoCheckBadValuePolicyHandling) {
    const size_t W = 50;
    const size_t L = 5;
    const double alpha = 0.5;

    MemoryKernelEstimator<double, NoCheckBadValuePolicy> estimator(W, L, alpha);

    // 包含NaN和Inf的序列
    std::vector<double> bad_value_sequence = {
        1.0, 2.0, std::numeric_limits<double>::quiet_NaN(),
        3.0, std::numeric_limits<double>::infinity(), 4.0
    };

    // 重复序列直到填满窗口
    for (size_t i = 0; i < W * 2; ++i) {
        double value = bad_value_sequence[i % bad_value_sequence.size()];
        // NoCheck策略应该总是返回true，不检查任何值
        EXPECT_TRUE(estimator.push(value));
    }

    // 由于包含NaN，记忆核应该为NaN（NaN传播）
    if (estimator.ready()) {
        double result = estimator.value();
        EXPECT_TRUE(std::isnan(result)) << "With NaN inputs and NoCheck policy, memory kernel should be NaN";
    }
}

/**
 * @brief 测试坏值处理策略 - SkipNaNInfPolicy
 * @brief 验证跳过策略会拒绝坏值（返回false）
 */
TEST_F(MemoryKernelTest, SkipNaNInfPolicyHandling) {
    const size_t W = 50;
    const size_t L = 5;
    const double alpha = 0.5;

    MemoryKernelEstimator<double, SkipNaNInfPolicy> estimator(W, L, alpha);

    // 测试包含坏值的序列
    std::vector<double> mixed_sequence = {
        1.0, 2.0, 3.0,  // 正常值
        std::numeric_limits<double>::quiet_NaN(),  // NaN
        4.0, 5.0,        // 正常值
        std::numeric_limits<double>::infinity(),   // Inf
        6.0              // 正常值
    };

    for (double value : mixed_sequence) {
        bool accepted = estimator.push(value);
        // NaN和Inf应该被拒绝（返回false），正常值应该被接受（返回true）
        if (std::isnan(value) || std::isinf(value)) {
            EXPECT_FALSE(accepted) << "Bad value should be rejected by SkipNaNInfPolicy";
        } else {
            EXPECT_TRUE(accepted) << "Normal value should be accepted";
        }
    }

    // 由于有坏值被跳过，可能需要更多数据来填满窗口
    // 填充足够的好数据以确保能计算出有效结果
    auto additional_data = generate_white_noise(0.0, 1.0, W + 10);
    for (double value : additional_data) {
        estimator.push(value);
    }

    // 应该能计算出有效的记忆核值（坏值被跳过）
    if (estimator.ready()) {
        double result = estimator.value();
        expect_valid_memory_kernel(result);
    }
}

/**
 * @brief 测试坏值处理策略 - ZeroNaNInfPolicy
 * @brief 验证零替换策略会将坏值替换为零，并总是返回true
 */
TEST_F(MemoryKernelTest, ZeroNaNInfPolicyHandling) {
    const size_t W = 50;
    const size_t L = 5;
    const double alpha = 0.5;

    MemoryKernelEstimator<double, ZeroNaNInfPolicy> estimator(W, L, alpha);

    // 包含坏值的序列
    std::vector<double> bad_sequence = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        1.0, 2.0, 3.0
    };

    for (double value : bad_sequence) {
        bool accepted = estimator.push(value);
        // ZeroNaNInfPolicy应该总是返回true（接受所有值，但替换坏值为0）
        EXPECT_TRUE(accepted) << "ZeroNaNInfPolicy should always accept values";
    }

    // 填充足够数据后应该能计算出结果
    auto additional_data = generate_white_noise(0.0, 1.0, W - bad_sequence.size());
    for (double value : additional_data) {
        estimator.push(value);
    }

    if (estimator.ready()) {
        double result = estimator.value();
        expect_valid_memory_kernel(result);
    }
}

/**
 * @brief 测试记忆核值的范围
 * @brief 验证记忆核值在理论上的合理范围内
 */
TEST_F(MemoryKernelTest, MemoryKernelValueRange) {
    const size_t W = 100;
    const size_t L = 10;
    const double alpha = 0.5;

    MemoryKernelEstimator<double> estimator(W, L, alpha);
    auto sequence = generate_ar1_sequence(0.5, 0.5, W + 20);

    for (double value : sequence) {
        estimator.push(value);
    }

    double memory_kernel = estimator.value();
    // 记忆核值应该在合理范围内，不是特别大
    EXPECT_LT(std::abs(memory_kernel), 10.0) << "Memory kernel should be within reasonable range";
}

/**
 * @brief 测试逐步填充数据的过程
 * @brief 验证随着数据逐渐增加，记忆核值的变化行为
 */
TEST_F(MemoryKernelTest, IncrementalDataFilling) {
    const size_t W = 50;
    const size_t L = 5;
    const double alpha = 0.5;

    MemoryKernelEstimator<double> estimator(W, L, alpha);
    auto sequence = generate_white_noise(0.0, 1.0, W * 2);

    // 逐步填充数据并观察行为
    for (size_t i = 0; i < sequence.size(); ++i) {
        estimator.push(sequence[i]);

        if (i < W) {
            // 在填满窗口前，应该不ready或返回NaN
            if (estimator.ready()) {
                double value = estimator.value();
                expect_valid_memory_kernel(value);
            } else {
                EXPECT_TRUE(std::isnan(estimator.value()));
            }
        } else {
            // 填满窗口后，应该总是ready并返回有效值
            EXPECT_TRUE(estimator.ready());
            double value = estimator.value();
            expect_valid_memory_kernel(value);
        }
    }
}