// test/utils/math/linear_algebra_test.cpp
#include <gtest/gtest.h>
#include <vector>
#include <deque>
#include "math/linear_algebra.h"

using namespace factorlib::math;

class LinearAlgebraTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置测试数据
        setup_test_data();
    }

    void setup_test_data() {
        // 2D数据: 3个样本，每个样本2个特征
        data_2d = {
            {1.0, 2.0},
            {2.0, 4.0},
            {3.0, 6.0}
        };

        // 3D数据: 4个样本，每个样本3个特征
        data_3d = {
            {1.0, 2.0, 3.0},
            {2.0, 3.0, 4.0},
            {3.0, 4.0, 5.0},
            {4.0, 5.0, 6.0}
        };

        // 单位矩阵 - 条件数为1，数值稳定
        identity_matrix = Eigen::Matrix3d::Identity();

        // 奇异矩阵 - 条件数为无穷大
        singular_matrix.resize(2, 2);
        singular_matrix << 1.0, 1.0,
                          1.0, 1.0;

        // 病态矩阵 - 条件数很大
        ill_conditioned_matrix.resize(2, 2);
        ill_conditioned_matrix << 1.0, 0.99,
                                0.99, 1.0;
    }

    std::vector<std::vector<double>> data_2d;
    std::vector<std::vector<double>> data_3d;
    Eigen::Matrix3d identity_matrix;
    Eigen::Matrix2d singular_matrix;
    Eigen::Matrix2d ill_conditioned_matrix;
};

// 测试协方差矩阵计算 - 空数据
TEST_F(LinearAlgebraTest, CovarianceMatrix_EmptyData) {
    // 测试空数据输入的情况
    std::vector<std::vector<double>> empty_data;
    auto result = LinearAlgebra<double>::covariance_matrix(empty_data);

    EXPECT_EQ(result.rows(), 0);
    EXPECT_EQ(result.cols(), 0);
}

// 测试协方差矩阵计算 - 单一样本
TEST_F(LinearAlgebraTest, CovarianceMatrix_SingleSample) {
    // 测试只有一个样本的情况，协方差矩阵应为零矩阵
    // 因为单一样本无法计算变量间的协同变化
    std::vector<std::vector<double>> single_sample = {{1.0, 2.0, 3.0}};

    auto result = LinearAlgebra<double>::covariance_matrix(single_sample);

    // 单一样本的协方差应该是零矩阵
    EXPECT_EQ(result.rows(), 3);
    EXPECT_EQ(result.cols(), 3);
    EXPECT_TRUE(result.isZero()) << "Single sample should produce zero covariance matrix";

    // 验证所有元素都是零
    for (int i = 0; i < result.rows(); ++i) {
        for (int j = 0; j < result.cols(); ++j) {
            EXPECT_DOUBLE_EQ(result(i, j), 0.0)
                << "Element (" << i << "," << j << ") should be zero";
        }
    }
}

// 新增测试：两个样本的情况（最小有效样本数）
TEST_F(LinearAlgebraTest, CovarianceMatrix_TwoSamples) {
    // 测试两个样本的情况，这是能够计算协方差的最小样本数
    std::vector<std::vector<double>> two_samples = {
        {1.0, 2.0},
        {3.0, 4.0}
    };

    auto result = LinearAlgebra<double>::covariance_matrix(two_samples);

    EXPECT_EQ(result.rows(), 2);
    EXPECT_EQ(result.cols(), 2);

    // 对于这两个样本，我们应该能得到非零的协方差
    // 样本1: (1,2), 样本2: (3,4)
    // 均值: (2,3)
    // 协方差矩阵应该是: [[2,2],[2,2]]
    EXPECT_NEAR(result(0, 0), 2.0, 1e-10);  // Var(X)
    EXPECT_NEAR(result(1, 1), 2.0, 1e-10);  // Var(Y)
    EXPECT_NEAR(result(0, 1), 2.0, 1e-10);  // Cov(X,Y)
    EXPECT_NEAR(result(1, 0), 2.0, 1e-10);  // Cov(Y,X) - 对称
}

// 测试协方差矩阵计算 - 2D数据
TEST_F(LinearAlgebraTest, CovarianceMatrix_2DData) {
    // 测试2D数据的协方差矩阵计算
    auto result = LinearAlgebra<double>::covariance_matrix(data_2d);

    // 验证矩阵维度
    EXPECT_EQ(result.rows(), 2);
    EXPECT_EQ(result.cols(), 2);

    // 验证对称性
    EXPECT_DOUBLE_EQ(result(0, 1), result(1, 0));

    // 对于线性相关的数据，协方差应该反映这种关系
    EXPECT_GT(result(0, 0), 0.0); // x方差为正
    EXPECT_GT(result(1, 1), 0.0); // y方差为正
}

// 测试协方差矩阵计算 - 不同容器类型
TEST_F(LinearAlgebraTest, CovarianceMatrix_DifferentContainerTypes) {
    // 测试使用不同容器类型（std::deque）的兼容性
    std::deque<std::deque<double>> deque_data = {
        {1.0, 2.0},
        {2.0, 4.0},
        {3.0, 6.0}
    };

    auto result = LinearAlgebra<double>::covariance_matrix(deque_data);

    EXPECT_EQ(result.rows(), 2);
    EXPECT_EQ(result.cols(), 2);
    EXPECT_GT(result(0, 0), 0.0);
}

// 测试协方差矩阵计算 - 不一致维度数据
TEST_F(LinearAlgebraTest, CovarianceMatrix_InconsistentDimensions) {
    // 测试维度不一致的数据应该抛出异常
    std::vector<std::vector<double>> inconsistent_data = {
        {1.0, 2.0},
        {3.0}  // 维度不一致
    };

    EXPECT_THROW(LinearAlgebra<double>::covariance_matrix(inconsistent_data),
                 std::invalid_argument);
}

// 测试条件数计算 - 单位矩阵
TEST_F(LinearAlgebraTest, ConditionNumber_IdentityMatrix) {
    // 测试单位矩阵的条件数，应该为1（最优条件数）
    double cond = LinearAlgebra<double>::condition_number(identity_matrix);

    EXPECT_DOUBLE_EQ(cond, 1.0);
}

// 测试条件数计算 - 奇异矩阵
TEST_F(LinearAlgebraTest, ConditionNumber_SingularMatrix) {
    // 测试奇异矩阵的条件数，应该为无穷大
    double cond = LinearAlgebra<double>::condition_number(singular_matrix);

    EXPECT_TRUE(std::isinf(cond));
}

// 测试条件数计算 - 病态矩阵
TEST_F(LinearAlgebraTest, ConditionNumber_IllConditionedMatrix) {
    // 测试病态矩阵的条件数，应该很大
    double cond = LinearAlgebra<double>::condition_number(ill_conditioned_matrix);

    EXPECT_GT(cond, 100.0); // 条件数应该很大
}

// 测试条件数计算 - 空矩阵
TEST_F(LinearAlgebraTest, ConditionNumber_EmptyMatrix) {
    // 测试空矩阵的条件数，应该返回0
    Eigen::MatrixXd empty_matrix(0, 0);
    double cond = LinearAlgebra<double>::condition_number(empty_matrix);

    EXPECT_DOUBLE_EQ(cond, 0.0);
}

// 测试协方差矩阵正则化
TEST_F(LinearAlgebraTest, RegularizeCovariance) {
    // 测试协方差矩阵正则化功能
    Eigen::Matrix2d original_cov;
    original_cov << 1.0, 0.5,
                   0.5, 1.0;

    double regularization = 0.1;
    auto regularized = LinearAlgebra<double>::regularize_covariance(original_cov, regularization);

    // 验证对角线元素增加了正则化参数
    EXPECT_DOUBLE_EQ(regularized(0, 0), original_cov(0, 0) + regularization);
    EXPECT_DOUBLE_EQ(regularized(1, 1), original_cov(1, 1) + regularization);

    // 非对角线元素保持不变
    EXPECT_DOUBLE_EQ(regularized(0, 1), original_cov(0, 1));
    EXPECT_DOUBLE_EQ(regularized(1, 0), original_cov(1, 0));
}

// 测试协方差矩阵正则化 - 非方阵异常
TEST_F(LinearAlgebraTest, RegularizeCovariance_NonSquareMatrix) {
    // 测试非方阵输入应该抛出异常
    Eigen::MatrixXd non_square(2, 3);
    non_square << 1, 2, 3, 4, 5, 6;

    EXPECT_THROW(LinearAlgebra<double>::regularize_covariance(non_square),
                 std::invalid_argument);
}

// 测试条件期望计算 - 基本功能
TEST_F(LinearAlgebraTest, ConditionalExpectation_Basic) {
    // 测试多元正态分布条件期望的基本计算
    Eigen::Vector3d mean;
    mean << 0.0, 0.0, 0.0;

    Eigen::Matrix3d covariance;
    covariance << 1.0, 0.5, 0.3,
                 0.5, 1.0, 0.4,
                 0.3, 0.4, 1.0;

    Eigen::Vector2d condition_values;
    condition_values << 1.0, 2.0;  // 前两个变量的观测值

    size_t target_index = 2;  // 预测第三个变量

    double result = LinearAlgebra<double>::conditional_expectation(
        mean, covariance, condition_values, target_index);

    // 验证结果合理性
    EXPECT_FALSE(std::isnan(result));
    EXPECT_FALSE(std::isinf(result));
}

// 测试条件期望计算 - 完美相关情况
TEST_F(LinearAlgebraTest, ConditionalExpectation_PerfectCorrelation) {
    // 测试完美线性相关情况下的条件期望
    Eigen::Vector2d mean;
    mean << 0.0, 0.0;

    Eigen::Matrix2d covariance;
    covariance << 1.0, 1.0,  // 完美正相关
                 1.0, 1.0;

    Eigen::VectorXd condition_values(1);
    condition_values << 2.0;  // 第一个变量的观测值

    size_t target_index = 1;  // 预测第二个变量

    double result = LinearAlgebra<double>::conditional_expectation(
        mean, covariance, condition_values, target_index);

    // 完美相关时，条件期望应该等于观测值
    EXPECT_NEAR(result, 2.0, 1e-10);
}

// 测试条件期望计算 - 参数检查异常
TEST_F(LinearAlgebraTest, ConditionalExpectation_ParameterValidation) {
    Eigen::Vector3d mean;
    mean << 0.0, 0.0, 0.0;

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

    Eigen::Vector2d valid_condition;
    valid_condition << 1.0, 2.0;

    // 测试非方阵协方差矩阵
    Eigen::MatrixXd non_square_cov(3, 2);
    EXPECT_THROW(LinearAlgebra<double>::conditional_expectation(
        mean, non_square_cov, valid_condition, 2), std::invalid_argument);

    // 测试维度不匹配
    Eigen::VectorXd wrong_size_condition(1);
    wrong_size_condition << 1.0;
    EXPECT_THROW(LinearAlgebra<double>::conditional_expectation(
        mean, covariance, wrong_size_condition, 2), std::invalid_argument);

    // 测试目标索引越界
    EXPECT_THROW(LinearAlgebra<double>::conditional_expectation(
        mean, covariance, valid_condition, 5), std::invalid_argument);
}

// 测试条件期望计算 - 独立变量情况
TEST_F(LinearAlgebraTest, ConditionalExpectation_IndependentVariables) {
    // 测试独立变量情况，条件期望应该等于无条件期望
    Eigen::Vector2d mean;
    mean << 1.0, 2.0;

    Eigen::Matrix2d covariance;
    covariance << 1.0, 0.0,  // 协方差为0，变量独立
                 0.0, 1.0;

    Eigen::VectorXd condition_values(1);
    condition_values << 5.0;  // 第一个变量的观测值

    size_t target_index = 1;  // 预测第二个变量

    double result = LinearAlgebra<double>::conditional_expectation(
        mean, covariance, condition_values, target_index);

    // 独立时，条件期望应该等于目标变量的无条件期望
    EXPECT_DOUBLE_EQ(result, mean(1));
}

// 测试整数类型支持
TEST_F(LinearAlgebraTest, IntegerTypeSupport) {
    // 测试整数类型数据的支持
    std::vector<std::vector<int>> int_data = {
        {1, 2},
        {3, 4},
        {5, 6}
    };

    auto result = LinearAlgebra<int>::covariance_matrix(int_data);

    EXPECT_EQ(result.rows(), 2);
    EXPECT_EQ(result.cols(), 2);
    EXPECT_GT(result(0, 0), 0.0);
}