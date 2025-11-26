#include "math/info_geometry.h"
#include <gtest/gtest.h>
#include <vector>
#include <cmath>

/**
 * @brief 信息几何测试夹具类
 *
 * 提供测试所需的通用工具方法和测试数据
 */
class InfoGeometryTest : public ::testing::Test {
protected:
    // 浮点数比较的容差
    const double tolerance = 1e-10;

    /**
     * @brief 设置测试环境
     */
    void SetUp() override {
        // 测试前的初始化代码可以放在这里
    }

    /**
     * @brief 清理测试环境
     */
    void TearDown() override {
        // 测试后的清理代码可以放在这里
    }

    /**
     * @brief 验证两个double值是否在容差范围内相等
     */
    bool doubleEqual(double a, double b) {
        return std::abs(a - b) < tolerance;
    }

    /**
     * @brief 验证double值是否为无穷大
     */
    bool isInfinity(double value) {
        return std::isinf(value);
    }
};

/**
 * @brief 测试伯努利分布Fisher信息的正常情况
 *
 * 测试用例：在合理参数范围内计算Fisher信息
 * 验证Fisher信息的数学性质和对称性
 */
TEST_F(InfoGeometryTest, Bernoulli_NormalCases) {
    // 测试对称性：θ和1-θ应该有相同的Fisher信息
    EXPECT_TRUE(doubleEqual(
        factorlib::math::fisher_bernoulli(0.3),
        factorlib::math::fisher_bernoulli(0.7)
    ));

    // 测试中点：θ=0.5时Fisher信息应该最小
    double fisher_05 = factorlib::math::fisher_bernoulli(0.5);
    double fisher_04 = factorlib::math::fisher_bernoulli(0.4);
    double fisher_06 = factorlib::math::fisher_bernoulli(0.6);
    EXPECT_LT(fisher_05, fisher_04);
    EXPECT_LT(fisher_05, fisher_06);

    // 验证具体数值计算正确性
    EXPECT_NEAR(factorlib::math::fisher_bernoulli(0.5), 4.0, tolerance);
    EXPECT_NEAR(factorlib::math::fisher_bernoulli(0.25), 1.0/(0.25*0.75), tolerance);
}

/**
 * @brief 测试伯努利分布Fisher信息的边界情况
 *
 * 测试用例：参数接近边界时的行为
 * 验证边界情况返回无穷大，但非常接近边界的值返回大数而非无穷大
 */
TEST_F(InfoGeometryTest, Bernoulli_BoundaryCases) {
    // 测试下边界
    EXPECT_TRUE(isInfinity(factorlib::math::fisher_bernoulli(0.0)));

    // 测试上边界
    EXPECT_TRUE(isInfinity(factorlib::math::fisher_bernoulli(1.0)));

    // 测试接近边界的情况：这些值不在边界上，所以不应该返回无穷大，而是很大的数
    EXPECT_FALSE(isInfinity(factorlib::math::fisher_bernoulli(1e-16)));
    EXPECT_FALSE(isInfinity(factorlib::math::fisher_bernoulli(1.0 - 1e-16)));
    // 验证这些很大的数确实很大
    EXPECT_GT(factorlib::math::fisher_bernoulli(1e-16), 1e15);
    EXPECT_GT(factorlib::math::fisher_bernoulli(1.0 - 1e-16), 1e15);

    // 测试非法参数
    EXPECT_TRUE(isInfinity(factorlib::math::fisher_bernoulli(-0.1)));
    EXPECT_TRUE(isInfinity(factorlib::math::fisher_bernoulli(1.1)));
}

/**
 * @brief 测试伯努利分布Fisher信息的极端值
 *
 * 测试用例：参数非常接近0或1时的数值稳定性
 */
TEST_F(InfoGeometryTest, Bernoulli_ExtremeValues) {
    // 测试非常小的正数
    double small_positive = 1e-10;
    double fisher_small = factorlib::math::fisher_bernoulli(small_positive);
    // 1/(1e-10 * (1-1e-10)) ≈ 1e10，所以应该大于1e9，小于1e11
    EXPECT_GT(fisher_small, 1e9);
    EXPECT_LT(fisher_small, 1e11);

    // 测试非常接近1的值
    double near_one = 1.0 - 1e-10;
    double fisher_near_one = factorlib::math::fisher_bernoulli(near_one);
    EXPECT_GT(fisher_near_one, 1e9);
    EXPECT_LT(fisher_near_one, 1e11);
}

/**
 * @brief 测试多项式分布Fisher信息的正常情况
 *
 * 测试用例：有效的概率分布向量
 * 验证对角近似的计算正确性
 */
TEST_F(InfoGeometryTest, Multinomial_NormalCases) {
    // 测试均匀分布情况
    std::vector<double> uniform_prob = {0.25, 0.25, 0.25, 0.25};
    auto result_uniform = factorlib::math::fisher_multinomial_diag(uniform_prob);

    EXPECT_EQ(result_uniform.size(), 4);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(result_uniform[i], 4.0, tolerance);
    }

    // 测试非均匀分布情况
    std::vector<double> skewed_prob = {0.5, 0.3, 0.2};
    auto result_skewed = factorlib::math::fisher_multinomial_diag(skewed_prob);

    EXPECT_EQ(result_skewed.size(), 3);
    EXPECT_NEAR(result_skewed[0], 2.0, tolerance);  // 1/0.5 = 2
    EXPECT_NEAR(result_skewed[1], 1.0/0.3, tolerance);
    EXPECT_NEAR(result_skewed[2], 5.0, tolerance);  // 1/0.2 = 5
}

/**
 * @brief 测试多项式分布Fisher信息的零概率情况
 *
 * 测试用例：包含零概率的情况
 * 验证零概率对应的Fisher信息为无穷大
 */
TEST_F(InfoGeometryTest, Multinomial_ZeroProbabilityCases) {
    // 测试包含零概率的情况
    std::vector<double> prob_with_zero = {0.4, 0.0, 0.6};
    auto result_with_zero = factorlib::math::fisher_multinomial_diag(prob_with_zero);

    EXPECT_EQ(result_with_zero.size(), 3);
    EXPECT_NEAR(result_with_zero[0], 2.5, tolerance);  // 1/0.4 = 2.5
    EXPECT_TRUE(isInfinity(result_with_zero[1]));      // 1/0.0 = ∞
    EXPECT_NEAR(result_with_zero[2], 1.0/0.6, tolerance);

    // 测试全零概率的情况（理论上不应该出现，但测试鲁棒性）
    std::vector<double> all_zeros = {0.0, 0.0, 0.0};
    auto result_all_zeros = factorlib::math::fisher_multinomial_diag(all_zeros);

    EXPECT_EQ(result_all_zeros.size(), 3);
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_TRUE(isInfinity(result_all_zeros[i]));
    }
}

/**
 * @brief 测试多项式分布Fisher信息的边界情况
 *
 * 测试用例：空向量和单元素向量
 * 验证函数对特殊输入的鲁棒性
 */
TEST_F(InfoGeometryTest, Multinomial_EdgeCases) {
    // 测试空向量
    std::vector<double> empty_prob;
    auto result_empty = factorlib::math::fisher_multinomial_diag(empty_prob);
    EXPECT_TRUE(result_empty.empty());

    // 测试单元素向量（退化为确定分布）
    std::vector<double> single_prob = {1.0};
    auto result_single = factorlib::math::fisher_multinomial_diag(single_prob);

    EXPECT_EQ(result_single.size(), 1);
    EXPECT_NEAR(result_single[0], 1.0, tolerance);

    // 测试单元素零概率
    std::vector<double> single_zero = {0.0};
    auto result_single_zero = factorlib::math::fisher_multinomial_diag(single_zero);

    EXPECT_EQ(result_single_zero.size(), 1);
    EXPECT_TRUE(isInfinity(result_single_zero[0]));
}

/**
 * @brief 测试多项式分布Fisher信息的原地计算版本
 *
 * 测试用例：验证原地计算与普通版本结果一致
 * 验证内存优化版本的功能正确性
 */
TEST_F(InfoGeometryTest, Multinomial_InplaceVersion) {
    // 准备测试数据
    std::vector<double> prob = {0.1, 0.2, 0.3, 0.4};
    std::vector<double> result_inplace(prob.size());

    // 调用原地计算版本
    factorlib::math::fisher_multinomial_diag_inplace(prob, result_inplace);

    // 调用普通版本进行对比
    auto result_normal = factorlib::math::fisher_multinomial_diag(prob);

    // 验证两个版本结果一致
    EXPECT_EQ(result_inplace.size(), result_normal.size());
    for (size_t i = 0; i < prob.size(); ++i) {
        EXPECT_NEAR(result_inplace[i], result_normal[i], tolerance);
    }

    // 验证具体数值正确性
    EXPECT_NEAR(result_inplace[0], 10.0, tolerance);  // 1/0.1 = 10
    EXPECT_NEAR(result_inplace[1], 5.0, tolerance);   // 1/0.2 = 5
    EXPECT_NEAR(result_inplace[2], 1.0/0.3, tolerance);
    EXPECT_NEAR(result_inplace[3], 2.5, tolerance);   // 1/0.4 = 2.5
}

/**
 * @brief 测试原地计算版本的容量处理
 *
 * 测试用例：输出向量容量不足的情况
 * 验证函数能正确处理向量大小不匹配的情况
 */
TEST_F(InfoGeometryTest, Multinomial_InplaceResize) {
    std::vector<double> prob = {0.3, 0.3, 0.4};
    std::vector<double> output_small(1); // 初始容量不足

    // 调用原地计算版本，应该会自动调整大小
    factorlib::math::fisher_multinomial_diag_inplace(prob, output_small);

    EXPECT_EQ(output_small.size(), 3);
    EXPECT_NEAR(output_small[0], 1.0/0.3, tolerance);
    EXPECT_NEAR(output_small[1], 1.0/0.3, tolerance);
    EXPECT_NEAR(output_small[2], 2.5, tolerance); // 1/0.4 = 2.5
}

/**
 * @brief 测试数值稳定性
 *
 * 测试用例：包含极小正概率的情况
 * 验证函数在数值极限情况下的行为
 */
TEST_F(InfoGeometryTest, NumericalStability) {
    // 测试包含极小正概率的情况
    std::vector<double> tiny_prob = {1e-10, 0.5, 0.5 - 1e-10};
    auto result_tiny = factorlib::math::fisher_multinomial_diag(tiny_prob);

    EXPECT_EQ(result_tiny.size(), 3);
    EXPECT_GT(result_tiny[0], 1e9); // 1/1e-10 应该是一个很大的数
    EXPECT_NEAR(result_tiny[1], 2.0, tolerance);
    EXPECT_GT(result_tiny[2], 2.0); // 1/(0.5-1e-10) 应该略大于2
}