#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "math/symbolic_dynamics.h"

using namespace factorlib::math;

// 测试用例 1：一般数值序列下，转移矩阵按行归一，拓扑熵 / 熵产生率可计算
// 入参：
//   - 符号边界 edges = [0, 1, 2, 3]（3 个符号，对应 3 个区间）；
//   - 滑窗长度 W = 8；
//   - 输入序列覆盖所有区间，并包含多种转移。
// 期望：
//   - transition_matrix 每一行的概率和≈1（对出现过转移的符号）；
//   - topological_entropy 与 entropy_production_rate 为有限数（非 NaN / 非 inf）。
TEST(SymbolicDynamicsTest, TransitionAndEntropyFinite) {
    std::vector<double> edges{0.0, 1.0, 2.0, 3.0};  // 3 个符号
    Symbolizer<double> sym(edges);
    RollingSymbolicDynamics<double> rsd(8, sym);

    // 构造一个覆盖所有符号、且有多种转移的序列
    std::vector<double> seq{
        0.1, 0.5,    // S0
        1.2, 1.8,    // S1
        2.4, 2.7,    // S2
        0.9, 0.4,    // S0
        1.1, 2.2     // S1 / S2
    };

    for (double x : seq) {
        rsd.push(x);
    }

    ASSERT_TRUE(rsd.ready());

    // 检查转移矩阵按行归一
    auto P = rsd.transition_matrix();
    for (const auto& row : P) {
        double s = 0.0;
        for (double v : row) s += v;
        if (s > 0.0) {
            // 对于实际出现过转移的符号，行和应该接近 1
            EXPECT_NEAR(s, 1.0, 1e-12);
        }
    }

    // 拓扑熵 / 熵产生率是有限数
    double htop  = rsd.topological_entropy();
    double sigma = rsd.entropy_production_rate();
    EXPECT_TRUE(std::isfinite(htop));
    EXPECT_TRUE(std::isfinite(sigma));
}

// 测试用例 2：确定性三态循环（S0→S1→S2→S0）应具有“零熵”
// 入参：
//   - 同样使用 edges = [0,1,2,3]，三个符号；
//   - 序列周期性往复：0.5(属 S0)、1.5(属 S1)、2.5(属 S2) 重复；
//   - 滑窗长度 W = 12，保证每个符号和转移在窗口内出现多次。
// 期望：
//   - topological_entropy ≈ 0（拓扑结构是简单的 3 点环）；
//   - entropy_production_rate ≈ 0（链是可逆 / 无净熵产生）。
TEST(SymbolicDynamicsTest, DeterministicCycleHasZeroEntropy) {
    std::vector<double> edges{0.0, 1.0, 2.0, 3.0};
    Symbolizer<double> sym(edges);
    RollingSymbolicDynamics<double> rsd(12, sym);

    // 构造 S0→S1→S2→S0 的确定性循环序列
    std::vector<double> seq;
    seq.reserve(60);
    for (int t = 0; t < 60; ++t) {
        int s = t % 3;
        if (s == 0)      seq.push_back(0.5);  // S0
        else if (s == 1) seq.push_back(1.5);  // S1
        else             seq.push_back(2.5);  // S2
    }

    for (double x : seq) {
        rsd.push(x);
    }

    ASSERT_TRUE(rsd.ready());

    double htop  = rsd.topological_entropy();
    double sigma = rsd.entropy_production_rate();

    // 对确定性周期，两个熵都应非常接近 0
    EXPECT_NEAR(htop,  0.0, 1e-6);
    EXPECT_NEAR(sigma, 0.0, 1e-6);
}
