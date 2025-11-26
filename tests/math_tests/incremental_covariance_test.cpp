#include "gtest/gtest.h"
#include "math/incremental_covariance.h"
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace factorlib::math;

/**
 * @class IncrementalCovarianceTest
 * @brief IncrementalCovariance 测试套件
 * 
 * 测试覆盖：
 * - 基本功能：数据添加、窗口管理
 * - 统计计算：均值向量、协方差矩阵
 * - 数值特性：维度兼容、数值稳定性
 * - 边界情况：空窗口、不足数据
 */
class IncrementalCovarianceTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 初始化3维测试数据
        data_3d = {
            Eigen::Vector3d(1.0, 2.0, 3.0),
            Eigen::Vector3d(2.0, 3.0, 4.0),
            Eigen::Vector3d(3.0, 4.0, 5.0),
            Eigen::Vector3d(4.0, 5.0, 6.0),
            Eigen::Vector3d(5.0, 6.0, 7.0)
        };
        window_size = 4;
    }

    void TearDown() override {
        // 清理资源
    }

    std::vector<Eigen::Vector3d> data_3d;
    std::size_t window_size;
};

/**
 * @test 测试多变量数据的基本处理
 * @brief 验证多变量数据的添加、窗口管理和基础统计
 * 
 * 维度验证：确保不同维度的数据正确处理
 * 窗口验证：滑动窗口大小限制有效
 */
TEST_F(IncrementalCovarianceTest, BasicMultivariateProcessing) {
    IncrementalCovariance<double, 3> cov_calculator(window_size);
    
    // 添加数据并验证窗口管理
    for (std::size_t i = 0; i < data_3d.size(); ++i) {
        cov_calculator.push(data_3d[i].cast<double>());
        
        EXPECT_LE(cov_calculator.size(), window_size);
        
        if (i >= window_size - 1) {
            EXPECT_TRUE(cov_calculator.is_window_full());
        }
    }
    
    EXPECT_EQ(cov_calculator.size(), window_size);
}

/**
 * @test 测试均值向量计算正确性
 * @brief 验证增量计算的均值向量与全量计算一致
 * 
 * 数学验证：μ = (1/n) * Σx_i
 * 分别用增量方法和全量方法计算，结果应该一致
 */
TEST_F(IncrementalCovarianceTest, MeanVectorCalculation) {
    IncrementalCovariance<double, 3> cov_calculator(window_size);
    
    // 添加数据
    for (const auto& vec : data_3d) {
        cov_calculator.push(vec.cast<double>());
    }
    
    // 获取增量计算的均值
    auto incremental_mean = cov_calculator.mean();
    
    // 全量计算均值验证
    Eigen::Vector3d full_mean = Eigen::Vector3d::Zero();
    auto current_data = data_3d;
    if (current_data.size() > window_size) {
        current_data.erase(current_data.begin(), 
                          current_data.end() - window_size);
    }
    
    for (const auto& vec : current_data) {
        full_mean += vec;
    }
    full_mean /= current_data.size();
    
    // 比较结果
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(incremental_mean(i), full_mean(i), 1e-10);
    }
}

/**
 * @test 测试协方差矩阵计算正确性
 * @brief 验证增量计算的协方差矩阵与全量计算一致
 *
 * 数学验证：Σ = 1/(n-1) * [Σ(x_i x_iᵀ) - n * μμᵀ]
 * 分别用增量方法和全量方法计算，结果应该一致
 *
 * 注意：使用无偏估计（样本协方差），除以(n-1)而不是n
 */
TEST_F(IncrementalCovarianceTest, CovarianceMatrixCalculation) {
    IncrementalCovariance<double, 3> cov_calculator(window_size);

    // 添加数据
    for (const auto& vec : data_3d) {
        cov_calculator.push(vec.cast<double>());
    }

    // 获取增量计算的协方差
    auto incremental_cov = cov_calculator.covariance();

    // 全量计算协方差验证（使用无偏估计）
    auto current_data = data_3d;
    if (current_data.size() > window_size) {
        current_data.erase(current_data.begin(),
                          current_data.end() - window_size);
    }

    // 计算均值
    Eigen::Vector3d full_mean = Eigen::Vector3d::Zero();
    for (const auto& vec : current_data) {
        full_mean += vec;
    }
    full_mean /= current_data.size();

    // 计算离差平方和
    Eigen::Matrix3d full_cov = Eigen::Matrix3d::Zero();
    for (const auto& vec : current_data) {
        Eigen::Vector3d centered = vec - full_mean;
        full_cov += centered * centered.transpose();
    }

    // 使用无偏估计：除以(n-1)而不是n
    if (current_data.size() > 1) {
        full_cov /= (current_data.size() - 1);
    } else {
        full_cov.setZero();  // 样本数不足，协方差为零矩阵
    }
    
    // 比较结果
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(incremental_cov(i, j), full_cov(i, j), 1e-10);
        }
    }
}

/**
 * @test 测试数值稳定性和坏值处理
 * @brief 验证对NaN/Inf数据的正确处理和数值稳定性
 * 
 * 鲁棒性验证：
 * - 包含坏值的数据应该被丢弃
 * - 好值数据应该正常处理
 * - 长时间运行不应出现数值问题
 */
TEST_F(IncrementalCovarianceTest, NumericalStabilityAndBadValueHandling) {
    IncrementalCovariance<double, 2> cov_calculator(5);
    
    // 混合好值和坏值
    std::vector<Eigen::Vector2d> mixed_data = {
        Eigen::Vector2d(1.0, 2.0),
        Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 3.0),  // 坏值
        Eigen::Vector2d(3.0, 4.0),
        Eigen::Vector2d(4.0, std::numeric_limits<double>::infinity()),  // 坏值
        Eigen::Vector2d(5.0, 6.0)
    };
    
    for (const auto& vec : mixed_data) {
        cov_calculator.push(vec);
    }
    
    // 只有好值被处理，应该能正常计算
    EXPECT_TRUE(cov_calculator.size() <= 3);  // 3个好值
    if (cov_calculator.size() >= 2) {
        auto cov = cov_calculator.covariance();
        EXPECT_TRUE(cov.allFinite());  // 结果应该是有限值
    }
}

/**
 * @test 测试不同维度兼容性
 * @brief 验证模板对不同维度的支持
 * 
 * 维度测试：确保1维、2维、高维数据都能正确处理
 * 特别注意：协方差计算使用无偏估计（除以n-1）
 */
TEST_F(IncrementalCovarianceTest, DifferentDimensionSupport) {
    // 测试1维情况
    IncrementalCovariance<double, 1> cov_1d(3);
    cov_1d.push(Eigen::Matrix<double, 1, 1>(1.0));
    cov_1d.push(Eigen::Matrix<double, 1, 1>(2.0));
    cov_1d.push(Eigen::Matrix<double, 1, 1>(3.0));

    auto mean_1d = cov_1d.mean();
    auto cov_1d_matrix = cov_1d.covariance();

    EXPECT_EQ(mean_1d.size(), 1);
    EXPECT_EQ(cov_1d_matrix.rows(), 1);
    EXPECT_EQ(cov_1d_matrix.cols(), 1);

    // 1维协方差就是方差
    EXPECT_NEAR(mean_1d(0), 2.0, 1e-10);

    // 修正：使用无偏方差（样本方差）计算
    // 数据 [1,2,3] 的样本方差 = [(1-2)² + (2-2)² + (3-2)²] / (3-1) = (1+0+1)/2 = 1.0
    EXPECT_NEAR(cov_1d_matrix(0, 0), 1.0, 1e-10);
}

/**
 * @test 测试清空和重置功能
 * @brief 验证clear()方法能正确重置所有状态
 */
TEST_F(IncrementalCovarianceTest, ClearAndResetFunctionality) {
    IncrementalCovariance<double, 3> cov_calculator(window_size);
    
    // 添加一些数据
    for (const auto& vec : data_3d) {
        cov_calculator.push(vec.cast<double>());
    }
    
    EXPECT_GT(cov_calculator.size(), 0);
    
    // 清空数据
    cov_calculator.clear();
    
    // 验证重置状态
    EXPECT_EQ(cov_calculator.size(), 0);
    EXPECT_FALSE(cov_calculator.is_window_full());
    
    // 均值应该是零向量
    auto zero_mean = cov_calculator.mean();
    for (int i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(zero_mean(i), 0.0);
    }
    
    // 协方差应该是零矩阵
    auto zero_cov = cov_calculator.covariance();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_DOUBLE_EQ(zero_cov(i, j), 0.0);
        }
    }
}

/**
 * @test 测试数据不足时的边界情况
 * @brief 验证当数据量不足时的合理行为
 * 
 * 边界情况验证：
 * - 空窗口：均值应为零向量，协方差应为零矩阵
 * - 单个数据：协方差应为零矩阵
 * - 两个数据：可以计算协方差
 */
TEST_F(IncrementalCovarianceTest, InsufficientDataCases) {
    IncrementalCovariance<double, 2> cov_calculator(5);
    
    // 测试空窗口
    auto empty_mean = cov_calculator.mean();
    auto empty_cov = cov_calculator.covariance();
    
    EXPECT_DOUBLE_EQ(empty_mean(0), 0.0);
    EXPECT_DOUBLE_EQ(empty_mean(1), 0.0);
    EXPECT_DOUBLE_EQ(empty_cov(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(empty_cov(1, 1), 0.0);
    
    // 测试单个数据点
    cov_calculator.push(Eigen::Vector2d(1.0, 2.0));
    auto single_cov = cov_calculator.covariance();
    EXPECT_DOUBLE_EQ(single_cov(0, 0), 0.0);  // 单个数据无法计算协方差
    
    // 测试两个数据点
    cov_calculator.push(Eigen::Vector2d(3.0, 4.0));
    auto two_cov = cov_calculator.covariance();
    EXPECT_GT(std::abs(two_cov(0, 0)), 0.0);  // 应该有非零协方差
}

/**
 * @test 验证协方差的有偏vs无偏计算
 * @brief 明确测试协方差计算使用的是无偏估计
 */
TEST_F(IncrementalCovarianceTest, UnbiasedCovarianceCalculation) {
    // 测试2维情况，更容易验证
    IncrementalCovariance<double, 2> cov_calc(3);

    // 简单数据：完全相关的两个变量
    cov_calc.push(Eigen::Vector2d(1.0, 2.0));
    cov_calc.push(Eigen::Vector2d(2.0, 4.0));
    cov_calc.push(Eigen::Vector2d(3.0, 6.0));

    auto cov = cov_calc.covariance();

    // 手动计算无偏协方差
    // x: [1,2,3], y: [2,4,6]
    // cov(x,y) = Σ(x_i - x̄)(y_i - ȳ) / (n-1)
    // = [(1-2)(2-4) + (2-2)(4-4) + (3-2)(6-4)] / 2
    // = [(-1)(-2) + 0 + (1)(2)] / 2 = (2 + 0 + 2)/2 = 2.0

    EXPECT_NEAR(cov(0, 1), 2.0, 1e-10);  // 协方差
    EXPECT_NEAR(cov(1, 0), 2.0, 1e-10);  // 对称性

    // 方差也应该使用无偏估计
    EXPECT_NEAR(cov(0, 0), 1.0, 1e-10);  // var(x) = [(1-2)²+(2-2)²+(3-2)²]/2 = (1+0+1)/2=1
    EXPECT_NEAR(cov(1, 1), 4.0, 1e-10);  // var(y) = [(2-4)²+(4-4)²+(6-4)²]/2 = (4+0+4)/2=4
}