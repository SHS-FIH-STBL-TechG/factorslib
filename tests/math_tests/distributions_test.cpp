// tests/math/distributions_test.cpp
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <deque>
#include <array>
#include <limits>

#include "math/distributions.h"

using namespace factorlib::math;

class DistributionsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 正态分布已知分位数参考值
        normal_ref_p_ = {0.001, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.95, 0.975, 0.99, 0.999};
        normal_ref_z_ = {-3.090232, -2.326348, -1.959964, -1.644854, -1.281552, -0.674490, 0.0,
                        0.674490, 1.281552, 1.644854, 1.959964, 2.326348, 3.090232};

        // 测试数据
        test_data_ = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
        test_data_with_nan_ = {1.0, 2.0, NAN, 4.0, 5.0, NAN, 7.0, 8.0, 9.0, 10.0};
    }

    std::vector<double> normal_ref_p_;
    std::vector<double> normal_ref_z_;
    std::vector<double> test_data_;
    std::vector<double> test_data_with_nan_;
};

// ============================================================================
// normal_quantile 测试
// ============================================================================

TEST_F(DistributionsTest, NormalQuantile_ValidInputs) {
    // 测试已知的分位数点，允许一定的数值误差
    for (size_t i = 0; i < normal_ref_p_.size(); ++i) {
        double computed = Distributions::normal_quantile(normal_ref_p_[i]);
        EXPECT_NEAR(computed, normal_ref_z_[i], 1e-4)
            << "Failed for p=" << normal_ref_p_[i];
    }
}

TEST_F(DistributionsTest, NormalQuantile_EdgeCases) {
    // 测试接近边界的值
    EXPECT_NO_THROW(Distributions::normal_quantile(1e-10));
    EXPECT_NO_THROW(Distributions::normal_quantile(1.0 - 1e-10));

    // 对称性测试
    double p1 = 0.25;
    double p2 = 0.75;
    double q1 = Distributions::normal_quantile(p1);
    double q2 = Distributions::normal_quantile(p2);
    EXPECT_NEAR(q1, -q2, 1e-10);
}

TEST_F(DistributionsTest, NormalQuantile_InvalidInputs) {
    // 边界值测试
    EXPECT_EQ(Distributions::normal_quantile(0.0), 0.0);
    EXPECT_EQ(Distributions::normal_quantile(1.0), 0.0);

    // 非法数值测试
    EXPECT_EQ(Distributions::normal_quantile(-0.5), 0.0);
    EXPECT_EQ(Distributions::normal_quantile(1.5), 0.0);

    // 非有限数值测试
    EXPECT_EQ(Distributions::normal_quantile(NAN), 0.0);
    EXPECT_EQ(Distributions::normal_quantile(std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_EQ(Distributions::normal_quantile(-std::numeric_limits<double>::infinity()), 0.0);
}

// ============================================================================
// normal_cdf 测试
// ============================================================================

TEST_F(DistributionsTest, NormalCDF_ValidInputs) {
    // 测试已知的CDF值
    EXPECT_NEAR(Distributions::normal_cdf(0.0), 0.5, 1e-10);
    EXPECT_NEAR(Distributions::normal_cdf(1.0), 0.8413447460, 1e-8);
    EXPECT_NEAR(Distributions::normal_cdf(-1.0), 0.1586552539, 1e-8);
    EXPECT_NEAR(Distributions::normal_cdf(1.644854), 0.95, 1e-4);
    EXPECT_NEAR(Distributions::normal_cdf(-1.644854), 0.05, 1e-4);
}

TEST_F(DistributionsTest, NormalCDF_DifferentFloatTypes) {
    // 测试模板支持的不同浮点类型
    EXPECT_NEAR(Distributions::normal_cdf(0.0f), 0.5f, 1e-6f);
    EXPECT_NEAR(Distributions::normal_cdf(0.0), 0.5, 1e-10);
    EXPECT_NEAR(Distributions::normal_cdf(0.0L), 0.5L, 1e-10L);
}

TEST_F(DistributionsTest, NormalCDF_InvalidInputs) {
    // 非有限数值测试
    EXPECT_EQ(Distributions::normal_cdf(NAN), 0.5);
    EXPECT_EQ(Distributions::normal_cdf(std::numeric_limits<double>::infinity()), 0.5);
    EXPECT_EQ(Distributions::normal_cdf(-std::numeric_limits<double>::infinity()), 0.5);
}

TEST_F(DistributionsTest, NormalCDF_Symmetry) {
    // 对称性测试
    double z_values[] = {0.5, 1.0, 1.5, 2.0, 2.5};
    for (double z : z_values) {
        double cdf_z = Distributions::normal_cdf(z);
        double cdf_neg_z = Distributions::normal_cdf(-z);
        EXPECT_NEAR(cdf_z + cdf_neg_z, 1.0, 1e-10) << "Failed for z=" << z;
    }
}

// ============================================================================
// empirical_inverse_cdf 测试
// ============================================================================

TEST_F(DistributionsTest, EmpiricalInverseCDF_ValidInputs) {
    // 测试标准分位数
    EXPECT_NEAR(Distributions::empirical_inverse_cdf(test_data_, 0.0), 1.0, 1e-10);
    EXPECT_NEAR(Distributions::empirical_inverse_cdf(test_data_, 0.5), 5.5, 1e-10); // 线性插值
    EXPECT_NEAR(Distributions::empirical_inverse_cdf(test_data_, 1.0), 10.0, 1e-10);

    // 测试25%和75%分位数
    EXPECT_NEAR(Distributions::empirical_inverse_cdf(test_data_, 0.25), 3.25, 1e-10);
    EXPECT_NEAR(Distributions::empirical_inverse_cdf(test_data_, 0.75), 7.75, 1e-10);
}

TEST_F(DistributionsTest, EmpiricalInverseCDF_DifferentContainers) {
    // 测试不同容器类型
    std::deque<double> deque_data = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::array<double, 5> array_data = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<float> float_data = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    std::vector<int> int_data = {1, 2, 3, 4, 5};

    EXPECT_NEAR(Distributions::empirical_inverse_cdf(deque_data, 0.5), 3.0, 1e-10);
    EXPECT_NEAR(Distributions::empirical_inverse_cdf(array_data, 0.5), 3.0, 1e-10);
    EXPECT_NEAR(Distributions::empirical_inverse_cdf(float_data, 0.5), 3.0, 1e-6);
    EXPECT_NEAR(Distributions::empirical_inverse_cdf(int_data, 0.5), 3.0, 1e-10);
}

TEST_F(DistributionsTest, EmpiricalInverseCDF_WithNaN) {
    // 测试包含NaN的数据
    double result = Distributions::empirical_inverse_cdf(test_data_with_nan_, 0.5);

    // 清洗后的数据应该是 [1,2,4,5,7,8,9,10]，中位数应该是6.0
    EXPECT_NEAR(result, 6.0, 1e-10);
}

TEST_F(DistributionsTest, EmpiricalInverseCDF_AllNaN) {
    // 测试全为NaN的情况
    std::vector<double> all_nan = {NAN, NAN, NAN};
    EXPECT_EQ(Distributions::empirical_inverse_cdf(all_nan, 0.5), 0.0);
}

TEST_F(DistributionsTest, EmpiricalInverseCDF_EmptyContainer) {
    // 测试空容器
    std::vector<double> empty_data;
    EXPECT_EQ(Distributions::empirical_inverse_cdf(empty_data, 0.5), 0.0);
}

TEST_F(DistributionsTest, EmpiricalInverseCDF_InvalidProbability) {
    // 测试非法概率值
    EXPECT_EQ(Distributions::empirical_inverse_cdf(test_data_, -0.1), 0.0);
    EXPECT_EQ(Distributions::empirical_inverse_cdf(test_data_, 1.1), 0.0);
    EXPECT_EQ(Distributions::empirical_inverse_cdf(test_data_, NAN), 0.0);
    EXPECT_EQ(Distributions::empirical_inverse_cdf(test_data_, std::numeric_limits<double>::infinity()), 0.0);
}

TEST_F(DistributionsTest, EmpiricalInverseCDF_SingleElement) {
    // 测试单元素容器
    std::vector<double> single_data = {42.0};
    EXPECT_EQ(Distributions::empirical_inverse_cdf(single_data, 0.0), 42.0);
    EXPECT_EQ(Distributions::empirical_inverse_cdf(single_data, 0.5), 42.0);
    EXPECT_EQ(Distributions::empirical_inverse_cdf(single_data, 1.0), 42.0);
}

TEST_F(DistributionsTest, EmpiricalInverseCDF_TwoElements) {
    // 测试双元素容器
    std::vector<double> two_data = {10.0, 20.0};
    EXPECT_EQ(Distributions::empirical_inverse_cdf(two_data, 0.0), 10.0);
    EXPECT_EQ(Distributions::empirical_inverse_cdf(two_data, 0.5), 15.0);
    EXPECT_EQ(Distributions::empirical_inverse_cdf(two_data, 1.0), 20.0);
}

// ============================================================================
// fisher_f_sf 测试
// ============================================================================

TEST_F(DistributionsTest, FisherSF_ValidInputs) {
    // 测试一些已知的F分布值
    // F(1,1) 在 F=1.0 时的右尾概率约为 0.5
    double p1 = fisher_f_sf(1.0, 1, 1);
    EXPECT_GT(p1, 0.0);
    EXPECT_LT(p1, 1.0);

    // 大F值应该有小的p值
    double p2 = fisher_f_sf(10.0, 1, 10);
    EXPECT_LT(p2, 0.05);

    // 小F值应该有大的p值
    double p3 = fisher_f_sf(0.1, 1, 10);
    EXPECT_GT(p3, 0.5);
}

TEST_F(DistributionsTest, FisherSF_DifferentFloatTypes) {
    // 测试不同浮点类型
    float p_float = fisher_f_sf(1.0f, 1, 1);
    double p_double = fisher_f_sf(1.0, 1, 1);
    long double p_long = fisher_f_sf(1.0L, 1, 1);

    EXPECT_GT(p_float, 0.0f);
    EXPECT_GT(p_double, 0.0);
    EXPECT_GT(p_long, 0.0L);
}

TEST_F(DistributionsTest, FisherSF_InvalidInputs) {
    // 测试非法输入
    EXPECT_EQ(fisher_f_sf(-1.0, 1, 1), 1.0);
    EXPECT_EQ(fisher_f_sf(NAN, 1, 1), 1.0);
    EXPECT_EQ(fisher_f_sf(std::numeric_limits<double>::infinity(), 1, 1), 1.0);

    // 测试非法自由度
    EXPECT_EQ(fisher_f_sf(1.0, 0, 1), 1.0);
    EXPECT_EQ(fisher_f_sf(1.0, -1, 1), 1.0);
    EXPECT_EQ(fisher_f_sf(1.0, 1, 0), 1.0);
    EXPECT_EQ(fisher_f_sf(1.0, 1, -1), 1.0);
}

TEST_F(DistributionsTest, FisherSF_ProbabilityBounds) {
    // 测试概率值在[0,1]范围内
    double p1 = fisher_f_sf(0.0, 1, 1);
    double p2 = fisher_f_sf(0.5, 1, 1);
    double p3 = fisher_f_sf(2.0, 1, 1);
    double p4 = fisher_f_sf(100.0, 1, 1);

    EXPECT_GE(p1, 0.0);
    EXPECT_LE(p1, 1.0);
    EXPECT_GE(p2, 0.0);
    EXPECT_LE(p2, 1.0);
    EXPECT_GE(p3, 0.0);
    EXPECT_LE(p3, 1.0);
    EXPECT_GE(p4, 0.0);
    EXPECT_LE(p4, 1.0);
}

TEST_F(DistributionsTest, FisherSF_Monotonicity) {
    // 测试单调性：F值越大，p值应该越小
    double p1 = fisher_f_sf(1.0, 2, 10);
    double p2 = fisher_f_sf(2.0, 2, 10);
    double p3 = fisher_f_sf(5.0, 2, 10);

    EXPECT_GT(p1, p2);
    EXPECT_GT(p2, p3);
}

// ============================================================================
// fisher_f_pvalue_right_tail 测试
// ============================================================================

TEST_F(DistributionsTest, FisherPValueRightTail_Alias) {
    // 测试别名函数是否与原始函数行为一致
    double p1 = fisher_f_sf(1.0, 1, 1);
    double p2 = fisher_f_pvalue_right_tail(1.0, 1, 1);

    EXPECT_EQ(p1, p2);
}

// ============================================================================
// 集成测试：多个函数的组合使用
// ============================================================================

TEST_F(DistributionsTest, IntegrationTest) {
    // 测试多个函数的组合使用
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};

    // 计算数据的90%分位数
    double quantile_90 = Distributions::empirical_inverse_cdf(data, 0.9);

    // 转换为z-score（假设我们知道均值和标准差）
    double mean = 3.0;
    double stddev = std::sqrt(2.0);
    double z_score = (quantile_90 - mean) / stddev;

    // 计算对应的正态分布概率
    double p_value = Distributions::normal_cdf(z_score);

    // 验证结果在合理范围内
    EXPECT_GT(p_value, 0.0);
    EXPECT_LT(p_value, 1.0);
}

// ============================================================================
// 性能测试（可选）
// ============================================================================

TEST_F(DistributionsTest, PerformanceTest) {
    // 测试大数据量下的性能
    const int N = 10000;
    std::vector<double> large_data(N);
    for (int i = 0; i < N; ++i) {
        large_data[i] = static_cast<double>(i);
    }

    // 应该能够快速处理
    EXPECT_NO_THROW({
        double result = Distributions::empirical_inverse_cdf(large_data, 0.95);
        EXPECT_GT(result, 0.0);
    });
}