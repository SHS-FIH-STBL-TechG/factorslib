// tests/rosenstein_lyapunov_test.cpp
#include <gtest/gtest.h>
#include <vector>
#include <cmath>

#include "math/rosenstein_lyapunov.h"

using factorlib::math::rosenstein_lyapunov;

// 本文件测试 Rosenstein 法估计最大李雅普诺夫指数：
//   - 对 logistic 映射 x_{t+1} = 4 x_t (1 - x_t)，理论最大 Lyapunov 指数为 ln 2；
//   - 在有限样本 + 嵌入 + 邻居搜索的近似下，估计值应接近 ln 2。

TEST(RosensteinLyapunovTest, LogisticMapApproxLn2)
{
    // 1) 生成 logistic 映射轨道
    const int    total_steps   = 2000;
    const int    burn_in       = 500;   // 丢弃前 500 步作为“热身”
    const double r             = 4.0;   // 完全混沌参数
    double       x0            = 0.123; // 初始条件

    std::vector<double> series;
    series.reserve(total_steps - burn_in);

    double x = x0;
    for (int i = 0; i < total_steps; ++i) {
        x = r * x * (1.0 - x);
        if (i >= burn_in) {
            series.push_back(x);
        }
    }

    // 2) 相空间重构 + Rosenstein 指数估计
    const int m       = 2;   // 嵌入维度
    const int tau     = 2;   // 时间延迟
    const int theiler = 10;  // Theiler 窗口：排除时间上过近的点，避免“伪自邻居”

    // 这里 fit_min/fit_max 作为“候选线性段”的大致范围；
    // 实现内部会在前若干步（1..min(fit_max,10)）上寻找近似线性增长区间。
    const int fit_min = 5;
    const int fit_max = 30;

    double got = rosenstein_lyapunov(series, m, tau, theiler, fit_min, fit_max);
    double exp = std::log(2.0);

    // 允许一定误差（有限样本 + 近似算法），但应该落在 [ln2 - 0.15, ln2 + 0.15] 之内。
    EXPECT_NEAR(got, exp, 0.15);
}
