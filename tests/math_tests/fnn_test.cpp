// tests/fnn_test.cpp
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <random>

#include "math/fnn.h"

using factorlib::math::fnn_ratio;

// 本文件测试 FNN（虚假最近邻）比例估计：
//   1. 对于一维平滑确定性序列（如正弦波），当嵌入维度从 m=1 提升到 m=2 时，
//      虚假最近邻比例应显著下降；
//   2. 对于白噪声序列，提升嵌入维度并不会显著降低虚假最近邻比例。

TEST(FNNTest, RatioDropsWhenEmbeddingDimEnoughForSine)
{
    // 用一个长正弦序列模拟“低维光滑动力系统”
    const int N = 1000;
    std::vector<double> x(N);
    for (int i = 0; i < N; ++i) {
        // 输入序列：x_t = sin(0.01 * t)
        x[i] = std::sin(0.01 * static_cast<double>(i));
    }

    const int    tau  = 2;      // 时间延迟
    const double Rtol = 10.0;   // 虚假最近邻判定阈值

    // r1: 在 m=1 维度下的虚假最近邻比例
    double r1 = fnn_ratio(x, 1, tau, Rtol);
    // r2: 在 m=2 维度下的虚假最近邻比例
    double r2 = fnn_ratio(x, 2, tau, Rtol);

    // 断言说明：
    //   - r1 应该比 r2 明显大（m=1 维度不足以展开相空间）；
    //   - r2 应该非常小（接近 0），因为 m=2 对正弦波来说已经足够。
    ASSERT_GT(r1, r2);
    ASSERT_LT(r2, 0.2);
}

TEST(FNNTest, WhiteNoiseKeepsHighRatio)
{
    // 生成一个高斯白噪声序列，用来模拟“高维/混沌”情形：
    // 在噪声下，相空间维度提升对“最近邻是否虚假”的改善有限。
    const int N = 1000;
    std::vector<double> x(N);

    // 简单的伪随机生成（测试中不要求严格统计性质）
    std::mt19937_64 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);
    for (int i = 0; i < N; ++i) {
        x[i] = dist(rng);
    }

    const int    tau  = 2;
    const double Rtol = 10.0;

    double r1 = fnn_ratio(x, 1, tau, Rtol);
    double r2 = fnn_ratio(x, 2, tau, Rtol);

    // 断言说明：
    //   - 对白噪声，m=1 和 m=2 下的 FNN 比例都应维持在较高水平；
    //   - 且 r2 不会像正弦波那样“骤降到接近 0”，通常仍然 > 0.4。
    ASSERT_GT(r1, 0.5);
    ASSERT_GT(r2, 0.4);
    // 允许 r2 略低于 r1，但不要求严格关系（不同随机种子可能轻微波动）。
}
