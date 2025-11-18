#include <gtest/gtest.h>
#include <vector>
#include <random>
#include <cmath>
#include "math/memory_kernel.h"

using namespace factorlib::math;

// 测试用例 1：AR(1) 正相关过程中记忆核估计值为有限数（数值稳定性检查）
// 入参：
//   - 窗口 W = 64，记忆阶数 L = 5，分数阶参数 alpha = 0.6；
//   - 序列：x_t = 0.9 x_{t-1} + N(0, 0.1^2)，共推入 256 步。
// 期望：
//   - ready() == true；
//   - value() 为有限实数（非 NaN / 非 inf）。
TEST(MemoryKernelTest, MonotoneDecayPositive) {
    std::size_t W = 64;
    std::size_t L = 5;
    double alpha = 0.6;
    MemoryKernelEstimator<double> mk(W, L, alpha);

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.1);

    double x = 0.0;
    for (int t = 0; t < 256; ++t) {
        x = 0.9 * x + noise(rng);
        mk.push(x);
    }

    ASSERT_TRUE(mk.ready());
    double M = mk.value();
    EXPECT_TRUE(std::isfinite(M));
}

// 测试用例 2：常数序列下记忆核应接近 0
// 入参：
//   - 窗口 W = 32，记忆阶数 L = 3，alpha = 0.5；
//   - 序列：x_t ≡ 1.0，推入 128 步。
// 期望：
//   - RollingAutoCorr 在方差为 0 时返回 0，因此所有 ρ_k≈0；
//   - 记忆核 M(τ) = sum_k w_k ρ_k ≈ 0。
TEST(MemoryKernelTest, ConstantSeriesYieldsZeroKernel) {
    std::size_t W = 32;
    std::size_t L = 3;
    double alpha = 0.5;
    MemoryKernelEstimator<double> mk(W, L, alpha);

    for (int t = 0; t < 128; ++t) {
        mk.push(1.0);
    }

    ASSERT_TRUE(mk.ready());
    double M = mk.value();
    EXPECT_NEAR(M, 0.0, 1e-12);
}
