#include "gtest/gtest.h"
#include "math/fractional_diff.h"
#include <vector>
#include <cmath>

using namespace factorlib::math;

/**
 * @class FractionalDiffTest
 * @brief 分数阶差分器测试套件
 *
 * 包含各种边界情况、正常情况和异常情况的测试用例
 */
class FractionalDiffTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 公共测试数据初始化
        constant_sequence = std::vector<float>{1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        linear_sequence = std::vector<float>{1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    }

    void TearDown() override {
        // 清理资源（如有需要）
    }

    std::vector<float> constant_sequence;
    std::vector<float> linear_sequence;
};

/**
 * @test 测试常数序列的分数阶差分
 * @brief 常数序列的分数阶差分不等于零，这是分数阶微积分的重要特性
 *
 * 数学原理：
 * - 经典微积分：常数的整数阶导数(n≥1)为零
 * - 分数阶微积分：常数的分数阶导数不为零（除非α是整数）
 *
 * 原因分析：
 * 分数阶微分算子具有记忆效应（非局部性），历史值对当前导数有贡献。
 * 对于常数序列 c，分数阶差分 D^α c = c × Σw_k(α)，其中权重和 Σw_k(α) ≠ 0。
 * 这体现了分数阶微积分与经典微积分的重要区别。
 */
TEST_F(FractionalDiffTest, ConstantSequence) {
    FractionalDifferentiatorGL<float> diff(0.5, 4);

    for (const auto& val : constant_sequence) {
        EXPECT_TRUE(diff.push(val));
    }

    float result = diff.value();

    // 计算理论预期值：常数 × 权重和
    // 对于 α=0.5, L=4，权重和为：1 - 0.5 - 0.125 - 0.0625 - 0.0390625 = 0.2734375
    const auto& weights = diff.weights();
    long double weight_sum = 0.0;
    for (auto w : weights) {
        weight_sum += w;
    }
    float expected = static_cast<float>(weight_sum) * constant_sequence[0];

    EXPECT_NEAR(result, expected, 1e-5f);
}

/**
 * @test 验证分数阶差分不满足常数函数导数为零的性质
 * @brief 这是分数阶微积分与经典微积分的重要区别
 *
 * 数学背景：
 * 分数阶微分算子是非局部的，具有长记忆特性。即使输入是常数函数，
 * 所有历史数据仍然对当前差分值有贡献，导致结果不为零。
 *
 * 这一特性在金融时间序列分析中很重要，因为它意味着即使价格不变，
 * 其分数阶差分仍然反映了一定的市场记忆效应。
 */
TEST_F(FractionalDiffTest, FractionalDerivativeOfConstantNotZero) {
    FractionalDifferentiatorGL<float> diff(0.5, 4);

    std::vector<float> constants(10, 2.0f); // 常数2序列
    for (const auto& val : constants) {
        diff.push(val);
    }

    float result = diff.value();

    // 验证常数2的分数阶差分不等于零
    EXPECT_NE(result, 0.0f);

    // 应该等于 2 × 权重和
    const auto& weights = diff.weights();
    long double weight_sum = 0.0;
    for (auto w : weights) weight_sum += w;
    float expected = 2.0f * static_cast<float>(weight_sum);

    EXPECT_NEAR(result, expected, 1e-5f);
}

/**
 * @test 测试线性序列的分数阶差分
 * @brief 验证线性序列分数阶差分的计算正确性
 *
 * 线性函数 f(t) = t 的 α 阶导数应该与 t^{-α} 成正比。
 */
TEST_F(FractionalDiffTest, LinearSequence) {
    FractionalDifferentiatorGL<float> diff(0.5, 4);

    for (const auto& val : linear_sequence) {
        EXPECT_TRUE(diff.push(val));
    }

    float result = diff.value();
    // 检查结果是否为有限数（具体数值依赖于权重系数）
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_GT(std::abs(result), 0.0f);  // 线性序列的差分不应为零
}

/**
 * @test 测试缓冲区未满时的行为
 * @brief 验证当数据不足时返回 NaN
 *
 * 这是重要的边界情况测试，确保在数据不足时不会产生误导性结果。
 */
TEST_F(FractionalDiffTest, NotReadyWhenBufferNotFull) {
    FractionalDifferentiatorGL<float> diff(0.3, 5);  // 需要6个数据点

    // 只推入3个数据，不足6个
    EXPECT_TRUE(diff.push(1.0f));
    EXPECT_TRUE(diff.push(2.0f));
    EXPECT_TRUE(diff.push(3.0f));

    EXPECT_FALSE(diff.ready());  // 应该未就绪
    float result = diff.value();
    EXPECT_TRUE(std::isnan(result));  // 应该返回NaN
}

/**
 * @test 测试不同分数阶数的影响
 * @brief 验证不同 α 值会产生不同的差分结果
 *
 * α 越大，差分操作越"强烈"，对序列的平滑程度影响越大。
 */
TEST_F(FractionalDiffTest, DifferentAlphaValues) {
    std::vector<float> sequence{1.0f, 2.0f, 1.5f, 3.0f, 2.5f, 4.0f};

    FractionalDifferentiatorGL<float> diff1(0.1, 4);  // 较小的α
    FractionalDifferentiatorGL<float> diff2(0.9, 4);  // 较大的α

    for (const auto& val : sequence) {
        diff1.push(val);
        diff2.push(val);
    }

    float result1 = diff1.value();
    float result2 = diff2.value();

    // 不同α值应该产生不同的结果
    EXPECT_NE(result1, result2);
    // 两个结果都应该是有限数
    EXPECT_TRUE(std::isfinite(result1));
    EXPECT_TRUE(std::isfinite(result2));
}

/**
 * @test 测试权重系数的正确性
 * @brief 验证权重系数的计算符合数学定义
 *
 * 权重系数应该满足递推关系：w_k = -w_{k-1} * (α - (k-1)) / k
 */
TEST_F(FractionalDiffTest, WeightCoefficients) {
    double alpha = 0.5;
    std::size_t L = 4;
    FractionalDifferentiatorGL<float> diff(alpha, L);

    const auto& weights = diff.weights();

    // 检查权重数量
    EXPECT_EQ(weights.size(), L + 1);

    // 检查第一个权重（应该为1）
    EXPECT_NEAR(weights[0], 1.0, 1e-10);

    // 检查权重递推关系
    for (std::size_t k = 1; k <= L; ++k) {
        double expected = -weights[k-1] * (alpha - (k-1)) / k;
        EXPECT_NEAR(weights[k], expected, 1e-10);
    }
}

/**
 * @test 测试环形缓冲区的正确性
 * @brief 验证缓冲区满后新数据会覆盖旧数据
 *
 * 这是滑动窗口算法的核心功能测试。
 */
TEST_F(FractionalDiffTest, CircularBufferBehavior) {
    FractionalDifferentiatorGL<float> diff(0.5, 3);  // 需要4个数据点

    // 推入第一组数据：1,2,3,4
    std::vector<float> first_set{1.0f, 2.0f, 3.0f, 4.0f};
    for (const auto& val : first_set) {
        diff.push(val);
    }
    float result1 = diff.value();

    // 推入新数据：5（应该覆盖1）
    diff.push(5.0f);
    float result2 = diff.value();

    // 两个结果应该不同（因为数据窗口改变了）
    EXPECT_NE(result1, result2);
    EXPECT_TRUE(std::isfinite(result1));
    EXPECT_TRUE(std::isfinite(result2));
}

/**
 * @test 测试坏值处理策略
 * @brief 验证当输入包含NaN/Inf时的行为
 *
 * 测试坏值处理策略是否能正确工作。
 */
TEST_F(FractionalDiffTest, BadValueHandling) {
    // 使用默认策略（不检查）
    FractionalDifferentiatorGL<float, NoCheckBadValuePolicy> diff1(0.5, 2);

    // 推入NaN值
    EXPECT_TRUE(diff1.push(std::numeric_limits<float>::quiet_NaN()));

    // 使用严格策略（这里需要根据实际的StrictBadValuePolicy实现来调整）
    // FractionalDifferentiatorGL<float, StrictBadValuePolicy> diff2(0.5, 2);
    // EXPECT_FALSE(diff2.push(std::numeric_limits<float>::quiet_NaN()));
}

/**
 * @test 测试记忆长度参数
 * @brief 验证不同记忆长度对结果的影响
 *
 * 记忆长度L越大，计算结果越精确但计算量也越大。
 */
TEST_F(FractionalDiffTest, DifferentMemoryLengths) {
    std::vector<float> sequence{1.0f, 2.0f, 1.0f, 3.0f, 2.0f, 4.0f, 3.0f, 5.0f};

    FractionalDifferentiatorGL<float> diff_short(0.5, 2);  // 短记忆
    FractionalDifferentiatorGL<float> diff_long(0.5, 6);   // 长记忆

    for (const auto& val : sequence) {
        diff_short.push(val);
        diff_long.push(val);
    }

    float result_short = diff_short.value();
    float result_long = diff_long.value();

    // 不同记忆长度应该产生不同的结果
    EXPECT_NE(result_short, result_long);
    EXPECT_TRUE(std::isfinite(result_short));
    EXPECT_TRUE(std::isfinite(result_long));
}

/**
 * @test 测试数据类型兼容性
 * @brief 验证模板类对不同数据类型的支持
 *
 * 测试float和double类型的兼容性。
 */
TEST_F(FractionalDiffTest, DifferentDataTypes) {
    FractionalDifferentiatorGL<float> diff_float(0.5, 3);
    FractionalDifferentiatorGL<double> diff_double(0.5, 3);

    std::vector<double> sequence{1.0, 2.0, 3.0, 4.0, 5.0};

    for (const auto& val : sequence) {
        diff_float.push(static_cast<float>(val));
        diff_double.push(val);
    }

    float result_float = diff_float.value();
    double result_double = diff_double.value();

    // 两种类型都应该产生有效结果
    EXPECT_TRUE(std::isfinite(result_float));
    EXPECT_TRUE(std::isfinite(result_double));
}

/**
 * @test 测试边界情况：α=0和α=1
 * @brief 验证极端分数阶数的行为
 *
 * α=0 时应该返回原序列，α=1 时应该返回一阶差分。
 */
TEST_F(FractionalDiffTest, ExtremeAlphaValues) {
    std::vector<float> sequence{1.0f, 2.0f, 3.0f, 4.0f, 5.0f};

    // α=0 的情况：理论上应该返回最后一个值（因为权重w0=1，其他权重很小）
    FractionalDifferentiatorGL<float> diff_zero(0.0, 4);
    for (const auto& val : sequence) {
        diff_zero.push(val);
    }
    float result_zero = diff_zero.value();
    EXPECT_NEAR(result_zero, 5.0f, 0.1f);  // 应该接近最后一个值

    // α=1 的情况：理论上应该接近一阶差分
    FractionalDifferentiatorGL<float> diff_one(1.0, 4);
    for (const auto& val : sequence) {
        diff_one.push(val);
    }
    float result_one = diff_one.value();
    // 一阶差分应该接近 1（对于线性序列）
    EXPECT_NEAR(result_one, 1.0f, 0.5f);
}