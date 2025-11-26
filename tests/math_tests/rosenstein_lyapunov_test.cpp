#pragma once

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <limits>
#include <random>
#include <numeric>

#include "math/rosenstein_lyapunov.h"

using namespace factorlib::math;

class RosensteinLyapunovTest : public ::testing::Test {
protected:
    static constexpr double kLooseTol = 0.5;   // 混沌系统估计用的宽松误差
    static constexpr double kTightTol = 0.2;   // 一般数值对比的稍紧误差

    // 生成 logistic 映射序列：x_{n+1} = r * x_n * (1 - x_n)
    std::vector<double> generate_logistic_series(int n, double r, double x0) {
        std::vector<double> x;
        x.reserve(n);
        x.push_back(x0);
        for (int i = 1; i < n; ++i) {
            double xn = x.back();
            x.push_back(r * xn * (1.0 - xn));
        }
        return x;
    }

    // 生成正弦序列：x_n = sin(2π n / period)
    std::vector<double> generate_sine_series(int n, double period) {
        std::vector<double> x;
        x.reserve(n);
        const double w = 2.0 * M_PI / period;
        for (int i = 0; i < n; ++i) {
            x.push_back(std::sin(w * i));
        }
        return x;
    }

    // 生成常数序列
    std::vector<double> generate_constant_series(int n, double value) {
        return std::vector<double>(n, value);
    }

    // 生成白噪声序列 U[0,1)
    std::vector<double> generate_uniform_noise(int n, unsigned seed = 42) {
        std::mt19937_64 rng(seed);
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        std::vector<double> x;
        x.reserve(n);
        for (int i = 0; i < n; ++i) {
            x.push_back(dist(rng));
        }
        return x;
    }

    // 朴素 Rosenstein 实现（不用 KD-Tree，O(N^2) 最近邻）
    // 与 math/rosenstein_lyapunov.h 中的函数保持逻辑一致，用于数值对比。
    double naive_rosenstein_lyapunov(const std::vector<double>& x,
                                     int m, int tau,
                                     int theiler,
                                     int fit_min, int fit_max) {
        auto Y = embed(x, m, tau);
        const int N = static_cast<int>(Y.size());
        if (N <= fit_max + 1) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        std::vector<int> nn(N, -1);

        // 1) 朴素最近邻搜索：先尝试满足 Theiler 窗口，再退化为只排除自身
        for (int i = 0; i < N; ++i) {
            double best_d2 = std::numeric_limits<double>::infinity();
            int best_j = -1;

            // 第一轮：|i-j| > theiler
            for (int j = 0; j < N; ++j) {
                if (j == i) continue;
                if (std::abs(i - j) <= theiler) continue;

                double d2 = 0.0;
                for (int d = 0; d < m; ++d) {
                    double diff = Y[i][d] - Y[j][d];
                    d2 += diff * diff;
                }
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_j = j;
                }
            }

            // 如果没找到满足 Theiler 的邻居，则允许只排除自身
            if (best_j < 0) {
                for (int j = 0; j < N; ++j) {
                    if (j == i) continue;
                    double d2 = 0.0;
                    for (int d = 0; d < m; ++d) {
                        double diff = Y[i][d] - Y[j][d];
                        d2 += diff * diff;
                    }
                    if (d2 < best_d2) {
                        best_d2 = d2;
                        best_j = j;
                    }
                }
            }

            nn[i] = best_j;
        }

        // 2) 计算平均 log 距离曲线
        std::vector<double> avg(fit_max + 1, 0.0);
        std::vector<int>    cnt(fit_max + 1, 0);

        for (int k = 1; k <= fit_max; ++k) {
            double s = 0.0;
            int    c = 0;
            for (int i = 0; i + k < N; ++i) {
                int j = nn[i];
                if (j < 0 || j + k >= N) continue;

                double d2_ij = 0.0;
                for (int d = 0; d < m; ++d) {
                    double diff = Y[i + k][d] - Y[j + k][d];
                    d2_ij += diff * diff;
                }
                if (d2_ij <= 0.0) continue;

                s += 0.5 * std::log(d2_ij);
                ++c;
            }
            if (c > 0) {
                avg[k] = s / c;
                cnt[k] = c;
            }
        }

        // 3) 与主实现相同：在 [1, min(fit_max,10)] 区间做最小二乘拟合
        double Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
        int    n  = 0;

        const int k_start = 1;
        const int k_end   = std::min(fit_max, 10);
        if (k_end - k_start + 1 < 2) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        for (int k = k_start; k <= k_end; ++k) {
            if (cnt[k] == 0) continue;
            double xk = static_cast<double>(k);
            double yk = avg[k];
            Sx  += xk;
            Sy  += yk;
            Sxx += xk * xk;
            Sxy += xk * yk;
            ++n;
        }
        if (n < 2) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        double slope = (n * Sxy - Sx * Sy) / (n * Sxx - Sx * Sx);
        return slope;
    }
};

// ==================== 嵌入函数 embed 的测试 ====================

/**
 * @test 嵌入维度与延迟的边界情况
 * @brief 验证 m<=0、tau<=0 或样本长度不足时，embed 返回空
 */
TEST_F(RosensteinLyapunovTest, EmbedInvalidParamsGiveEmpty) {
    std::vector<double> x{1.0, 2.0, 3.0};

    auto Y1 = embed(x, 0, 1);
    auto Y2 = embed(x, 2, 0);
    auto Y3 = embed(x, 3, 2); // 需要长度 >= (m-1)*tau+1 = 5，当前 x 只有 3

    EXPECT_TRUE(Y1.empty());
    EXPECT_TRUE(Y2.empty());
    EXPECT_TRUE(Y3.empty());
}

/**
 * @test 正常嵌入结果验证
 * @brief 验证 embed 的形状与内容是否正确
 */
TEST_F(RosensteinLyapunovTest, EmbedShapeAndValues) {
    // 序列 0,1,2,3,4,5
    std::vector<double> x{0, 1, 2, 3, 4, 5};

    int m   = 3;
    int tau = 1;
    // 预期：N = len(x) - (m-1)*tau = 6 - 2 = 4 个嵌入点
    // Y[0] = [0,1,2]
    // Y[1] = [1,2,3]
    // Y[2] = [2,3,4]
    // Y[3] = [3,4,5]
    auto Y = embed(x, m, tau);

    ASSERT_EQ(Y.size(), 4u);
    for (const auto& vec : Y) {
        EXPECT_EQ(vec.size(), static_cast<size_t>(m));
    }

    EXPECT_DOUBLE_EQ(Y[0][0], 0.0);
    EXPECT_DOUBLE_EQ(Y[0][1], 1.0);
    EXPECT_DOUBLE_EQ(Y[0][2], 2.0);

    EXPECT_DOUBLE_EQ(Y[3][0], 3.0);
    EXPECT_DOUBLE_EQ(Y[3][1], 4.0);
    EXPECT_DOUBLE_EQ(Y[3][2], 5.0);

    // 再测试一个 tau > 1 的情况
    // x = 0,1,2,3,4,5,6,7；m=3,tau=2 => N = 8 - (3-1)*2 = 4
    // Y[0] = [0,2,4], Y[1] = [1,3,5], Y[2] = [2,4,6], Y[3] = [3,5,7]
    std::vector<double> x2{0,1,2,3,4,5,6,7};
    auto Y2 = embed(x2, 3, 2);
    ASSERT_EQ(Y2.size(), 4u);
    EXPECT_DOUBLE_EQ(Y2[0][0], 0.0);
    EXPECT_DOUBLE_EQ(Y2[0][1], 2.0);
    EXPECT_DOUBLE_EQ(Y2[0][2], 4.0);
    EXPECT_DOUBLE_EQ(Y2[3][0], 3.0);
    EXPECT_DOUBLE_EQ(Y2[3][1], 5.0);
    EXPECT_DOUBLE_EQ(Y2[3][2], 7.0);
}

// ==================== rosenstein_lyapunov 参数与边界行为 ====================

/**
 * @test 样本不足时返回 NaN
 * @brief 验证在嵌入后点数太少（N <= fit_max + 1）时，函数返回 NaN
 */
TEST_F(RosensteinLyapunovTest, NotEnoughSamplesReturnsNaN) {
    std::vector<double> x{0.1, 0.2, 0.3, 0.4};

    int m       = 3;
    int tau     = 1;
    int theiler = 2;
    int fit_min = 1;
    int fit_max = 5;  // 嵌入后 N 会很小，触发 N <= fit_max + 1 分支

    double lambda = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);
    EXPECT_TRUE(std::isnan(lambda));
}

/**
 * @test fit_max 过大时返回 NaN
 * @brief 当 fit_max 远大于 N 时，拟合样本不足导致 NaN
 */
TEST_F(RosensteinLyapunovTest, TooLargeFitMaxReturnsNaN) {
    auto x = generate_logistic_series(200, 4.0, 0.123456);

    int m       = 3;
    int tau     = 1;
    int theiler = 10;
    int fit_min = 1;
    int fit_max = 500; // 明显大于样本量

    double lambda = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);
    EXPECT_TRUE(std::isnan(lambda));
}

/**
 * @test 常数序列的李雅普诺夫指数
 * @brief 对于完全常数的序列，所有嵌入点重合，距离为 0，理论上无法估计，应返回 NaN
 */
TEST_F(RosensteinLyapunovTest, ConstantSeriesReturnsNaN) {
    auto x = generate_constant_series(1000, 0.5);

    int m       = 3;
    int tau     = 1;
    int theiler = 10;
    int fit_min = 1;
    int fit_max = 15;

    double lambda = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);
    EXPECT_TRUE(std::isnan(lambda));
}

/**
 * @test 过大的 Theiler 窗口行为
 * @brief 当 Theiler 窗口非常大时，算法会退化为只排除自身的最近邻，但仍应给出有限结果
 */
TEST_F(RosensteinLyapunovTest, TooLargeTheilerWindowStillFinite) {
    auto x = generate_logistic_series(1000, 4.0, 0.123456);

    int m       = 3;
    int tau     = 1;
    int theiler = 900;   // 非常大的 Theiler 窗口
    int fit_min = 1;
    int fit_max = 15;

    double lambda = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);

    // 应该返回一个有限的数值，而不是 NaN / Inf
    ASSERT_TRUE(std::isfinite(lambda));
    // 简单做个 sanity check：不应该离谱地大或小
    EXPECT_GT(lambda, -5.0);
    EXPECT_LT(lambda,  5.0);
}

// ==================== 数值正确性：与朴素实现对比 ====================

/**
 * @test logistic 映射：KD-Tree 实现 vs 朴素实现
 * @brief 在中等长度的序列上，对比 rosenstein_lyapunov 与 naive 实现的结果
 */
TEST_F(RosensteinLyapunovTest, LogisticMapKdTreeMatchesNaive) {
    const int N = 2000;
    auto x = generate_logistic_series(N, 4.0, 0.234567);

    int m       = 5;
    int tau     = 1;
    int theiler = 50;
    int fit_min = 1;
    int fit_max = 20;

    double lambda_kd  = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);
    double lambda_ref = naive_rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);

    ASSERT_TRUE(std::isfinite(lambda_kd) || std::isnan(lambda_kd));
    ASSERT_TRUE(std::isfinite(lambda_ref) || std::isnan(lambda_ref));

    // 两者都为 NaN：接受
    if (std::isnan(lambda_ref)) {
        EXPECT_TRUE(std::isnan(lambda_kd));
    } else {
        // 两个实现结果应比较接近
        EXPECT_NEAR(lambda_kd, lambda_ref, kTightTol);
    }
}

/**
 * @test 正弦序列：KD-Tree 实现 vs 朴素实现
 * @brief 对于非混沌的周期序列，两种实现应给出接近的结果（接近 0）
 */
TEST_F(RosensteinLyapunovTest, SineSeriesKdTreeMatchesNaive) {
    const int N = 2000;
    auto x = generate_sine_series(N, 100.0);

    int m       = 5;
    int tau     = 1;
    int theiler = 50;
    int fit_min = 1;
    int fit_max = 20;

    double lambda_kd  = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);
    double lambda_ref = naive_rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);

    ASSERT_TRUE(std::isfinite(lambda_kd) || std::isnan(lambda_kd));
    ASSERT_TRUE(std::isfinite(lambda_ref) || std::isnan(lambda_ref));

    if (std::isnan(lambda_ref)) {
        EXPECT_TRUE(std::isnan(lambda_kd));
    } else {
        EXPECT_NEAR(lambda_kd, lambda_ref, kTightTol);
        EXPECT_NEAR(lambda_kd, 0.0, 0.3);
    }
}

// ==================== 数值性质：与理论/直觉的一致性 ====================

/**
 * @test logistic 映射 (r=4) 的最大李雅普诺夫指数
 * @brief 对于 logistic 映射 x_{n+1} = 4 x_n (1 - x_n)，理论最大 LLE = ln 2 ≈ 0.693
 */
TEST_F(RosensteinLyapunovTest, LogisticMapR4LyapunovNearLog2) {
    const int N = 4000;
    auto x = generate_logistic_series(N, 4.0, 0.123456);

    int m       = 5;
    int tau     = 1;
    int theiler = 50;
    int fit_min = 1;
    int fit_max = 20;

    double lambda = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);

    const double expected = std::log(2.0);  // ≈ 0.693147...

    ASSERT_TRUE(std::isfinite(lambda));
    EXPECT_GT(lambda, 0.1);                 // 必须明显大于 0（混沌系统）
    EXPECT_LT(lambda, 1.5);                 // 不应太离谱
    EXPECT_NEAR(lambda, expected, kLooseTol);
}

/**
 * @test logistic 映射较小参数 (r=3) 的 Lyapunov 指数较小
 * @brief 对 r=3 的 logistic 映射，系统更“温和”，LLE 应明显小于 r=4 情况
 */
TEST_F(RosensteinLyapunovTest, LogisticMapR3LyapunovSmallerThanR4) {
    const int N = 4000;
    auto x3 = generate_logistic_series(N, 3.0, 0.123456);
    auto x4 = generate_logistic_series(N, 4.0, 0.223456);

    int m       = 5;
    int tau     = 1;
    int theiler = 50;
    int fit_min = 1;
    int fit_max = 20;

    double lambda3 = rosenstein_lyapunov(x3, m, tau, theiler, fit_min, fit_max);
    double lambda4 = rosenstein_lyapunov(x4, m, tau, theiler, fit_min, fit_max);

    ASSERT_TRUE(std::isfinite(lambda3));
    ASSERT_TRUE(std::isfinite(lambda4));

    EXPECT_LT(lambda3, lambda4);
}

/**
 * @test 周期性正弦序列的 Lyapunov 指数
 * @brief 对于简单周期序列（非混沌），最大 Lyapunov 指数应接近 0
 */
TEST_F(RosensteinLyapunovTest, PeriodicSineSeriesLyapunovNearZero) {
    const int N = 4000;
    auto x = generate_sine_series(N, 100.0);

    int m       = 5;
    int tau     = 1;
    int theiler = 50;
    int fit_min = 1;
    int fit_max = 20;

    double lambda = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);

    ASSERT_TRUE(std::isfinite(lambda));
    EXPECT_NEAR(lambda, 0.0, 0.3);
}

/**
 * @test 白噪声序列的 Lyapunov 指数
 * @brief 对 i.i.d. 噪声序列，几何结构接近“云团”，估计的 LLE 不应明显为负，
 *        同时也不应极端大。
 */
TEST_F(RosensteinLyapunovTest, WhiteNoiseLyapunovRoughlySmallPositive) {
    const int N = 4000;
    auto x = generate_uniform_noise(N, 123);

    int m       = 5;
    int tau     = 1;
    int theiler = 50;
    int fit_min = 1;
    int fit_max = 20;

    double lambda = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);

    ASSERT_TRUE(std::isfinite(lambda));
    EXPECT_GT(lambda, -0.5);   // 不太可能很负
    EXPECT_LT(lambda, 1.5);    // 也不应非常大
}

// ==================== Theiler 窗口与拟合区间的影响 ====================

/**
 * @test 不同 Theiler 窗口下的稳定性
 * @brief 验证在合理范围内调整 theiler 时，估计结果变化不应过大
 */
TEST_F(RosensteinLyapunovTest, DifferentTheilerValuesDoNotDriftTooMuch) {
    const int N = 4000;
    auto x = generate_logistic_series(N, 4.0, 0.345678);

    int m       = 5;
    int tau     = 1;
    int fit_min = 1;
    int fit_max = 20;

    double lambda_t0  = rosenstein_lyapunov(x, m, tau, 0,   fit_min, fit_max);
    double lambda_t20 = rosenstein_lyapunov(x, m, tau, 20,  fit_min, fit_max);
    double lambda_t50 = rosenstein_lyapunov(x, m, tau, 50,  fit_min, fit_max);

    ASSERT_TRUE(std::isfinite(lambda_t0));
    ASSERT_TRUE(std::isfinite(lambda_t20));
    ASSERT_TRUE(std::isfinite(lambda_t50));

    // 不同 theiler 下的估计值应在一个合理范围内波动
    EXPECT_NEAR(lambda_t0,  lambda_t20, 0.5);
    EXPECT_NEAR(lambda_t20, lambda_t50, 0.5);
}

/**
 * @test 不同 fit_max 的影响
 * @brief 验证在适度改变拟合上限步数时，估计结果不应剧烈变化
 */
TEST_F(RosensteinLyapunovTest, DifferentFitMaxValuesDoNotDriftTooMuch) {
    const int N = 4000;
    auto x = generate_logistic_series(N, 4.0, 0.456789);

    int m       = 5;
    int tau     = 1;
    int theiler = 50;
    int fit_min = 1;

    double lambda_f10 = rosenstein_lyapunov(x, m, tau, theiler, fit_min, 10);
    double lambda_f15 = rosenstein_lyapunov(x, m, tau, theiler, fit_min, 15);
    double lambda_f20 = rosenstein_lyapunov(x, m, tau, theiler, fit_min, 20);

    ASSERT_TRUE(std::isfinite(lambda_f10));
    ASSERT_TRUE(std::isfinite(lambda_f15));
    ASSERT_TRUE(std::isfinite(lambda_f20));

    EXPECT_NEAR(lambda_f10, lambda_f15, 0.4);
    EXPECT_NEAR(lambda_f15, lambda_f20, 0.4);
}

// ==================== 稳定性 / “性能” 相关测试 ====================

/**
 * @test 长序列下的稳定性
 * @brief 对 logistic 映射增加样本长度，验证估计值相对稳定且不崩溃
 */
TEST_F(RosensteinLyapunovTest, LogisticMapLengthStability) {
    auto x_short = generate_logistic_series(3000, 4.0, 0.345678);
    auto x_long  = generate_logistic_series(8000, 4.0, 0.345678);

    int m       = 5;
    int tau     = 1;
    int theiler = 50;
    int fit_min = 1;
    int fit_max = 20;

    double lambda_short = rosenstein_lyapunov(x_short, m, tau, theiler, fit_min, fit_max);
    double lambda_long  = rosenstein_lyapunov(x_long,  m, tau, theiler, fit_min, fit_max);

    ASSERT_TRUE(std::isfinite(lambda_short));
    ASSERT_TRUE(std::isfinite(lambda_long));

    // 序列变长后，估计值不应该剧烈变化
    EXPECT_NEAR(lambda_short, lambda_long, 0.3);
}
