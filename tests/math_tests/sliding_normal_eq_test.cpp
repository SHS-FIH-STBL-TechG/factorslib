#pragma once

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <limits>
#include <random>

#include <Eigen/Dense>
#include "math/sliding_normal_eq.h"

using namespace factorlib::math;

class SlidingNormalEqTest : public ::testing::Test {
protected:
    using Solver = SlidingNormalEq<double>;
    using Row    = Solver::Row;

    static constexpr double kTightTol = 1e-10;
    static constexpr double kLooseTol = 1e-2;

    // 辅助函数：构造 Row 向量
    Row make_row(const std::vector<double>& vals) {
        Row r(static_cast<int>(vals.size()));
        for (int i = 0; i < static_cast<int>(vals.size()); ++i) {
            r(i) = vals[i];
        }
        return r;
    }

    // 辅助函数：朴素 OLS（使用 Eigen 直接构造 X、y 做一次性 QR 求解）
    // 用于和 SlidingNormalEq::solve 的结果做数值对比
    Row naive_ols(const std::vector<Row>& Xrows,
                  const std::vector<double>& yvals,
                  double& RSS_out) {
        const int n = static_cast<int>(Xrows.size());
        const int d = static_cast<int>(Xrows[0].size());

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> X(n, d);
        Eigen::Matrix<double, Eigen::Dynamic, 1>              y(n);

        for (int i = 0; i < n; ++i) {
            X.row(i) = Xrows[i].transpose();
            y(i)     = yvals[i];
        }

        Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> qr(X);
        Row beta = qr.solve(y);

        Eigen::Matrix<double, Eigen::Dynamic, 1> e = y - X * beta;
        RSS_out = e.squaredNorm();
        return beta;
    }
};

// ==================== 基础功能测试 ====================

/**
 * @test 构造参数与基本属性
 * @brief 验证 dim / window / 初始 size / reset 行为
 */
TEST_F(SlidingNormalEqTest, ConstructorAndBasicProperties) {
    const int d = 3;
    const int W = 10;

    Solver solver(d, W);

    EXPECT_EQ(solver.dim(), d);
    EXPECT_EQ(solver.window(), W);
    EXPECT_EQ(solver.size(), 0);

    // push 一些样本后 reset
    Row x(d);
    x << 1.0, 2.0, 3.0;
    solver.push(x, 1.0);
    solver.push(x, 2.0);

    EXPECT_EQ(solver.size(), 2);

    solver.reset();

    EXPECT_EQ(solver.size(), 0);
    Row beta(d);
    double RSS = 0.0;
    EXPECT_FALSE(solver.solve(beta, RSS));  // 空窗口下 solve 应返回 false
}

/**
 * @test 滑动窗口尺寸控制
 * @brief 验证 push 超过窗口后，size 始终不超过 window，且样本数正确
 */
TEST_F(SlidingNormalEqTest, SlidingWindowSizeControl) {
    const int d = 2;
    const int W = 5;
    Solver solver(d, W);

    Row x(d);
    x << 1.0, 2.0;

    // 连续 push 10 条数据，应最终 size == W
    for (int i = 0; i < 10; ++i) {
        Row xi(d);
        xi << 1.0, static_cast<double>(i);
        solver.push(xi, static_cast<double>(i));
        int expected_size = std::min(W, i + 1);
        EXPECT_EQ(solver.size(), expected_size);
    }

    EXPECT_EQ(solver.size(), W);
}

// ==================== 数值正确性测试：精确场景 ====================

/**
 * @test 一维线性关系（无截距）精确拟合
 * @brief y = 2x，验证回归系数精确为 2，RSS 接近 0
 */
TEST_F(SlidingNormalEqTest, OneDimExactFitWithoutIntercept) {
    const int d = 1;
    const int W = 20;
    Solver solver(d, W);

    // y = 2x
    for (int i = 1; i <= 20; ++i) {
        Row x(d);
        x(0) = static_cast<double>(i);
        double y = 2.0 * x(0);
        solver.push(x, y);
    }

    Row beta(d);
    double RSS = 0.0;
    ASSERT_TRUE(solver.solve(beta, RSS));

    EXPECT_NEAR(beta(0), 2.0, kTightTol);
    EXPECT_NEAR(RSS, 0.0, 1e-12);
}

/**
 * @test 二维线性关系（含截距）精确拟合
 * @brief y = a + b * t，使用 x = [1, t] 作为自变量
 */
TEST_F(SlidingNormalEqTest, TwoDimExactFitWithIntercept) {
    const int d = 2;
    const int W = 50;
    Solver solver(d, W);

    const double a = 3.0;
    const double b = 2.0;

    // t = 0..49
    for (int t = 0; t < 50; ++t) {
        Row x(d);
        x(0) = 1.0;                  // 截距项
        x(1) = static_cast<double>(t);
        double y = a + b * x(1);
        solver.push(x, y);
    }

    Row beta(d);
    double RSS = 0.0;
    ASSERT_TRUE(solver.solve(beta, RSS));

    EXPECT_NEAR(beta(0), a, 1e-9);
    EXPECT_NEAR(beta(1), b, 1e-9);
    EXPECT_NEAR(RSS, 0.0, 1e-9);
}

/**
 * @test 过定约束场景 + 高斯噪声
 * @brief y = a + b * t + 噪声，用较大的窗口，检查估计接近真实参数
 */
TEST_F(SlidingNormalEqTest, OverdeterminedWithNoise) {
    const int d = 2;
    const int W = 200;
    Solver solver(d, W);

    const double a_true = 1.5;
    const double b_true = -0.7;

    std::mt19937_64 rng(42);
    std::normal_distribution<double> noise(0.0, 0.1);

    for (int t = 0; t < W; ++t) {
        Row x(d);
        x(0) = 1.0;
        x(1) = static_cast<double>(t);
        double y = a_true + b_true * x(1) + noise(rng);
        solver.push(x, y);
    }

    Row beta(d);
    double RSS = 0.0;
    ASSERT_TRUE(solver.solve(beta, RSS));

    EXPECT_NEAR(beta(0), a_true, 0.05);  // 有噪声，给一点空间
    EXPECT_NEAR(beta(1), b_true, 0.05);
    EXPECT_GE(RSS, 0.0);
}

/**
 * @test 欠定系统的最小二乘解
 * @brief n < d 时，系统欠定，但 QR 仍会给出一个最小范数解，残差应可接受
 */
TEST_F(SlidingNormalEqTest, UnderdeterminedSystem) {
    const int d = 3;
    const int W = 5;
    Solver solver(d, W);

    // 只有两条样本，但维度是 3
    // 例如真实模型：y = x0 + 2*x1 + 0*x2
    {
        Row x(d);
        x << 1.0, 2.0, 0.5;
        double y = x(0) + 2.0 * x(1); // 忽略 x2
        solver.push(x, y);
    }
    {
        Row x(d);
        x << 0.0, -1.0, 3.0;
        double y = x(0) + 2.0 * x(1);
        solver.push(x, y);
    }

    Row beta(d);
    double RSS = 0.0;
    ASSERT_TRUE(solver.solve(beta, RSS));

    // 欠定情况下解不唯一，但应至少能很好拟合训练样本
    EXPECT_GE(RSS, 0.0);
    EXPECT_LT(RSS, 1e-10); // 拟合误差几乎为 0
}

/**
 * @test 共线特征的稳健性
 * @brief 两个特征高度共线（x2 = 2 * x1），检查拟合结果在训练集上残差很小
 */
TEST_F(SlidingNormalEqTest, CollinearFeaturesStillFitTrainingData) {
    const int d = 2;
    const int W = 30;
    Solver solver(d, W);

    // 构造共线特征：x2 = 2 * x1
    // 真正的目标仅依赖 x1：y = 3 * x1
    for (int i = 0; i < W; ++i) {
        double x1 = static_cast<double>(i);
        double x2 = 2.0 * x1;
        Row x(d);
        x << x1, x2;
        double y = 3.0 * x1;
        solver.push(x, y);
    }

    Row beta(d);
    double RSS = 0.0;
    ASSERT_TRUE(solver.solve(beta, RSS));

    // 解可能不唯一，但训练残差应非常小
    EXPECT_NEAR(RSS, 0.0, 1e-8);
}

// ==================== 滑动窗口行为测试 ====================

/**
 * @test 滑窗滚动后的系数稳定性（简单线性）
 * @brief y = 2x，在滑动窗口滚动过程中，估计斜率应始终接近 2
 */
TEST_F(SlidingNormalEqTest, SlidingWindowSlopeStability) {
    const int d = 1;
    const int W = 10;
    Solver solver(d, W);

    std::vector<double> slopes;

    for (int i = 0; i < 50; ++i) {
        Row x(d);
        x(0) = static_cast<double>(i);
        double y = 2.0 * x(0);
        solver.push(x, y);

        if (solver.size() == W) {
            Row beta(d);
            double RSS = 0.0;
            ASSERT_TRUE(solver.solve(beta, RSS));
            slopes.push_back(beta(0));
            EXPECT_NEAR(RSS, 0.0, 1e-8);
        }
    }

    // 应有多次估计结果
    EXPECT_GT(slopes.size(), 10u);

    // 所有估计斜率都应接近 2
    for (double s : slopes) {
        EXPECT_NEAR(s, 2.0, 1e-9);
    }
}

/**
 * @test 滑窗仅使用最近 N 个样本
 * @brief 前后两组数据完全不同，检查解只取决于最近窗口内的数据
 */
TEST_F(SlidingNormalEqTest, SlidingWindowUsesOnlyRecentSamples) {
    const int d = 1;
    const int W = 5;
    Solver solver(d, W);

    // 第一段：y = 2x
    for (int i = 0; i < W; ++i) {
        Row x(d);
        x(0) = static_cast<double>(i);
        double y = 2.0 * x(0);
        solver.push(x, y);
    }

    Row beta1(d);
    double RSS1 = 0.0;
    ASSERT_TRUE(solver.solve(beta1, RSS1));

    // 第二段：y = -3x，用另一批样本完全覆盖窗口
    for (int i = 0; i < W; ++i) {
        Row x(d);
        x(0) = static_cast<double>(i + 100); // 避免与前面重复
        double y = -3.0 * x(0);
        solver.push(x, y);
    }

    Row beta2(d);
    double RSS2 = 0.0;
    ASSERT_TRUE(solver.solve(beta2, RSS2));

    // 验证当前解不再是 2，而接近 -3
    EXPECT_NEAR(beta1(0), 2.0, 1e-9);
    EXPECT_NEAR(beta2(0), -3.0, 1e-9);
}

// ==================== 坏值处理测试 ====================

/**
 * @test 含 NaN 的样本被丢弃
 * @brief 验证 x 或 y 含 NaN 时样本不会进入滑窗，size 不变，解不受影响
 */
TEST_F(SlidingNormalEqTest, SampleWithNaNIsDropped) {
    const int d = 1;
    const int W = 5;
    Solver solver(d, W);

    // 先放入 3 个正常样本
    for (int i = 0; i < 3; ++i) {
        Row x(d);
        x(0) = static_cast<double>(i + 1);
        double y = 2.0 * x(0);
        solver.push(x, y);
    }
    EXPECT_EQ(solver.size(), 3);

    // 含 NaN 的样本（x）
    {
        Row x(d);
        x(0) = std::numeric_limits<double>::quiet_NaN();
        solver.push(x, 1.0);
    }
    EXPECT_EQ(solver.size(), 3);  // size 不应变化

    // 含 NaN 的样本（y）
    {
        Row x(d);
        x(0) = 10.0;
        double y = std::numeric_limits<double>::quiet_NaN();
        solver.push(x, y);
    }
    EXPECT_EQ(solver.size(), 3);

    // 继续加入正常样本，直到窗口满
    for (int i = 3; i < W; ++i) {
        Row x(d);
        x(0) = static_cast<double>(i + 1);
        double y = 2.0 * x(0);
        solver.push(x, y);
    }
    EXPECT_EQ(solver.size(), W);

    Row beta(d);
    double RSS = 0.0;
    ASSERT_TRUE(solver.solve(beta, RSS));

    EXPECT_NEAR(beta(0), 2.0, 1e-9);
    EXPECT_NEAR(RSS, 0.0, 1e-8);
}

/**
 * @test 含 Inf 的样本被丢弃
 * @brief 验证 x 或 y 含 Inf 时样本被丢弃
 */
TEST_F(SlidingNormalEqTest, SampleWithInfIsDropped) {
    const int d = 1;
    const int W = 5;
    Solver solver(d, W);

    // 初始 3 个正常样本
    for (int i = 0; i < 3; ++i) {
        Row x(d);
        x(0) = static_cast<double>(i + 1);
        double y = -1.0 * x(0);
        solver.push(x, y);
    }
    EXPECT_EQ(solver.size(), 3);

    // x 含 Inf
    {
        Row x(d);
        x(0) = std::numeric_limits<double>::infinity();
        solver.push(x, -1.0);
    }
    EXPECT_EQ(solver.size(), 3);

    // y 含 Inf
    {
        Row x(d);
        x(0) = 10.0;
        double y = std::numeric_limits<double>::infinity();
        solver.push(x, y);
    }
    EXPECT_EQ(solver.size(), 3);

    // 填满窗口
    for (int i = 3; i < W; ++i) {
        Row x(d);
        x(0) = static_cast<double>(i + 1);
        double y = -1.0 * x(0);
        solver.push(x, y);
    }
    EXPECT_EQ(solver.size(), W);

    Row beta(d);
    double RSS = 0.0;
    ASSERT_TRUE(solver.solve(beta, RSS));

    EXPECT_NEAR(beta(0), -1.0, 1e-9);
    EXPECT_NEAR(RSS, 0.0, 1e-8);
}

// ==================== 与朴素 OLS 的对比测试 ====================

/**
 * @test 与朴素一次性 QR OLS 的对比
 * @brief 在固定窗口内，比较 SlidingNormalEq::solve 与 naive OLS 的系数和 RSS
 */
TEST_F(SlidingNormalEqTest, CompareWithNaiveOls) {
    const int d = 3;
    const int W = 50;
    Solver solver(d, W);

    std::vector<Row>    Xrows;
    std::vector<double> yvals;

    std::mt19937_64 rng(123);
    std::normal_distribution<double> noise(0.0, 0.05);

    // 真实模型：y = 1.0 + 2.0*x1 - 0.5*x2
    for (int i = 0; i < W; ++i) {
        double x1 = static_cast<double>(i);
        double x2 = std::sin(0.1 * i);
        double x3 = std::cos(0.2 * i); // 未出现在真实模型中

        Row x(d);
        x << 1.0, x1, x2; // 这里 x3 不入模，只用 x1,x2，加个截距
        double y = 1.0 + 2.0 * x1 - 0.5 * x2 + noise(rng);

        solver.push(x, y);
        Xrows.push_back(x);
        yvals.push_back(y);
    }

    // SlidingNormalEq 解
    Row beta_solver(d);
    double RSS_solver = 0.0;
    ASSERT_TRUE(solver.solve(beta_solver, RSS_solver));

    // 朴素一次性 OLS 解
    double RSS_naive = 0.0;
    Row beta_naive = naive_ols(Xrows, yvals, RSS_naive);

    // 数值上应非常接近
    for (int i = 0; i < d; ++i) {
        EXPECT_NEAR(beta_solver(i), beta_naive(i), 1e-8);
    }
    EXPECT_NEAR(RSS_solver, RSS_naive, 1e-6);
}

// ==================== 稳定性 / 小性能测试 ====================

/**
 * @test 长时间连续 push + solve 的稳定性
 * @brief 在较长序列上多次滑动回归，验证不会产生 NaN / Inf，且斜率估计大致合理
 */
TEST_F(SlidingNormalEqTest, LongRunStability) {
    const int d = 2;
    const int W = 30;
    Solver solver(d, W);

    std::mt19937_64 rng(2025);
    std::normal_distribution<double> noise(0.0, 0.1);

    const double a_true = 0.7;
    const double b_true = 1.3;

    std::vector<double> slopes;

    for (int t = 0; t < 1000; ++t) {
        Row x(d);
        x(0) = 1.0;
        x(1) = static_cast<double>(t);
        double y = a_true + b_true * x(1) + noise(rng);

        solver.push(x, y);

        if (solver.size() == W) {
            Row beta(d);
            double RSS = 0.0;
            ASSERT_TRUE(solver.solve(beta, RSS));

            ASSERT_TRUE(beta.allFinite());
            ASSERT_TRUE(std::isfinite(RSS));
            EXPECT_GE(RSS, 0.0);

            // 记录斜率，用于后面统计
            slopes.push_back(beta(1));
        }
    }

    // 应该有不少斜率估计
    ASSERT_GT(slopes.size(), 10u);

    // 计算斜率的平均值，应该接近真实 b_true
    double mean_slope = 0.0;
    for (double s : slopes) {
        mean_slope += s;
    }
    mean_slope /= static_cast<double>(slopes.size());

    EXPECT_NEAR(mean_slope, b_true, 0.3);  // 给一个比较宽松但合理的范围
}

