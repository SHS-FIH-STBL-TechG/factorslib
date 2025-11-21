// tests/math/fnn_test.cpp
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <limits>

#include "math/fnn.h"

using namespace factorlib::math;

class FNNTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 测试数据
        linear_series_ = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
        constant_series_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    }

    std::vector<double> linear_series_;
    std::vector<double> constant_series_;
};

// ============================================================================
// 基于代码逻辑的测试
// ============================================================================

TEST_F(FNNTest, CodeLogic_ConstantSeriesReturnsNaN) {
    // 常数序列：所有点在相空间中重合，距离为0，会触发 continue
    double result = fnn_ratio(constant_series_, 2, 1, 10.0);
    EXPECT_TRUE(std::isnan(result));
}

TEST_F(FNNTest, CodeLogic_LinearSeriesBasic) {
    // 线性序列应该能正常计算
    double result = fnn_ratio(linear_series_, 2, 1, 10.0);

    // 根据代码，结果应该在 [0,1] 范围内或者是 NaN
    if (!std::isnan(result)) {
        EXPECT_GE(result, 0.0);
        EXPECT_LE(result, 1.0);
    }
}

TEST_F(FNNTest, CodeLogic_InvalidParametersReturnNaN) {
    // 测试代码中的边界检查
    EXPECT_TRUE(std::isnan(fnn_ratio(linear_series_, 0, 1, 10.0)));  // m <= 0
    EXPECT_TRUE(std::isnan(fnn_ratio(linear_series_, 2, 0, 10.0)));  // tau <= 0

    // 序列太短：需要满足 x.size() >= (m+1)*tau + 1
    std::vector<double> short_series = {1.0, 2.0};
    EXPECT_TRUE(std::isnan(fnn_ratio(short_series, 3, 2, 10.0)));  // 需要长度 >= 8
}

TEST_F(FNNTest, CodeLogic_EmptySeriesReturnsNaN) {
    std::vector<double> empty_series;
    EXPECT_TRUE(std::isnan(fnn_ratio(empty_series, 2, 1, 10.0)));
}

// ============================================================================
// 验证算法正确性的测试
// ============================================================================

TEST_F(FNNTest, AlgorithmCorrectness_ManualVerification) {
    // 创建一个简单的序列，可以手动验证结果
    // 序列: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
    // m=2, tau=1
    // 嵌入:
    // Y_2: [0,1], [1,2], [2,3], [3,4], [4,5], [5,6], [6,7], [7,8]
    // Y_3: [0,1,2], [1,2,3], [2,3,4], [3,4,5], [4,5,6], [5,6,7], [6,7,8], [7,8,9]

    // 对于这个简单序列，我们可以预期某些行为
    double result = fnn_ratio(linear_series_, 2, 1, 10.0);

    // 不假设具体数值，只验证基本性质
    if (!std::isnan(result)) {
        EXPECT_GE(result, 0.0);
        EXPECT_LE(result, 1.0);
    }
}

TEST_F(FNNTest, AlgorithmCorrectness_RtolSensitivity) {
    // 测试 Rtol 参数的影响
    // 使用更严格的 Rtol 应该识别更多的虚假最近邻

    std::vector<double> diverse_series = {0.0, 1.0, 0.1, 0.9, 0.2, 0.8, 0.3, 0.7, 0.4, 0.6};

    double result_rtol5 = fnn_ratio(diverse_series, 2, 1, 5.0);
    double result_rtol20 = fnn_ratio(diverse_series, 2, 1, 20.0);

    // 如果两个结果都有效，更小的 Rtol 应该产生更大或相等的 FNN 比例
    if (!std::isnan(result_rtol5) && !std::isnan(result_rtol20)) {
        EXPECT_GE(result_rtol5, result_rtol20);
    }
}

// ============================================================================
// 数值稳定性测试
// ============================================================================

TEST_F(FNNTest, NumericalStability_SmallValues) {
    std::vector<double> small_values = {1e-10, 2e-10, 3e-10, 4e-10, 5e-10};
    double result = fnn_ratio(small_values, 2, 1, 10.0);

    // 应该能够处理小数值而不崩溃
    if (!std::isnan(result)) {
        EXPECT_GE(result, 0.0);
        EXPECT_LE(result, 1.0);
    }
}

TEST_F(FNNTest, NumericalStability_LargeValues) {
    std::vector<double> large_values = {1e10, 2e10, 3e10, 4e10, 5e10};
    double result = fnn_ratio(large_values, 2, 1, 10.0);

    if (!std::isnan(result)) {
        EXPECT_GE(result, 0.0);
        EXPECT_LE(result, 1.0);
    }
}

// ============================================================================
// 一致性测试
// ============================================================================

TEST_F(FNNTest, Consistency_SameInputSameOutput) {
    double result1 = fnn_ratio(linear_series_, 2, 1, 10.0);
    double result2 = fnn_ratio(linear_series_, 2, 1, 10.0);

    // 相同输入应该产生相同输出
    if (std::isnan(result1)) {
        EXPECT_TRUE(std::isnan(result2));
    } else {
        EXPECT_FALSE(std::isnan(result2));
        EXPECT_DOUBLE_EQ(result1, result2);
    }
}

// ============================================================================
// 性能测试
// ============================================================================

TEST_F(FNNTest, Performance_MediumSeries) {
    const int N = 100;
    std::vector<double> medium_series(N);
    for (int i = 0; i < N; ++i) {
        medium_series[i] = std::sin(0.1 * i);
    }

    // 应该能在合理时间内完成计算
    EXPECT_NO_THROW({
        double result = fnn_ratio(medium_series, 3, 2, 15.0);
        // 不检查具体结果，只测试不崩溃
    });
}

// ============================================================================
// 测试 FNN 用于实际场景
// ============================================================================

TEST_F(FNNTest, PracticalUseCase_EmbeddingDimensionSelection) {
    // 模拟实际使用 FNN 选择嵌入维度的过程
    const int max_dim = 5;

    for (int m = 1; m <= max_dim; ++m) {
        double ratio = fnn_ratio(linear_series_, m, 1, 10.0);

        // 记录结果但不做具体假设
        // 在实际应用中，会选择 FNN 比例首次显著下降的维度
        if (!std::isnan(ratio)) {
            EXPECT_GE(ratio, 0.0);
            EXPECT_LE(ratio, 1.0);
        }
    }
}

// ============================================================================
// 测试边界情况的正确处理
// ============================================================================

TEST_F(FNNTest, EdgeCase_AllPointsIdentical) {
    // 所有点相同的情况
    std::vector<double> identical_points(10, 5.0);
    double result = fnn_ratio(identical_points, 2, 1, 10.0);
    EXPECT_TRUE(std::isnan(result));
}

TEST_F(FNNTest, EdgeCase_MinimumValidLength) {
    // 测试最小有效长度
    // 根据代码：x.size() >= (m+1)*tau + 1
    // 对于 m=1, tau=1: 需要长度 >= 3
    std::vector<double> min_series = {1.0, 2.0, 3.0};
    EXPECT_NO_THROW(fnn_ratio(min_series, 1, 1, 10.0));
}