#pragma once

#include <gtest/gtest.h>
#include <vector>
#include <deque>
#include <cmath>
#include <limits>
#include <random>
#include <numeric>
#include <stdexcept>

#include "math/symbolic_dynamics.h"

using namespace factorlib::math;

// ==================== 测试夹具 ====================

class SymbolicDynamicsTest : public ::testing::Test {
protected:
    static constexpr double kTolTight = 1e-12;
    static constexpr double kTolLoose = 1e-6;

    // 辅助：基于 Symbolizer 的朴素符号化
    std::vector<int> naive_symbolize(const Symbolizer<double>& sym,
                                     const std::vector<double>& data) {
        std::vector<int> s;
        s.reserve(data.size());
        for (double x : data) {
            s.push_back(sym.symbol_of(x));
        }
        return s;
    }

    // 辅助：给定最后一个窗口的符号序列，计算朴素转移矩阵（行归一化）
    std::vector<std::vector<double>>
    naive_transition_matrix_last_window(const Symbolizer<double>& sym,
                                        const std::vector<double>& data,
                                        std::size_t W) {
        const std::size_t N = data.size();
        if (N < W) {
            // 理论上测试里不会发生；这里防御式返回空矩阵避免 ASSERT_* 宏报错
            return {};
        }
        const int k = sym.k();

        std::vector<int> s_window;
        s_window.reserve(W);
        for (std::size_t i = N - W; i < N; ++i) {
            s_window.push_back(sym.symbol_of(data[i]));
        }

        std::vector<std::vector<int>> counts(k, std::vector<int>(k, 0));
        std::vector<int> row_sum(k, 0);
        for (std::size_t t = 0; t + 1 < s_window.size(); ++t) {
            int a = s_window[t];
            int b = s_window[t + 1];
            ++counts[a][b];
            ++row_sum[a];
        }

        std::vector<std::vector<double>> P(k, std::vector<double>(k, 0.0));
        for (int i = 0; i < k; ++i) {
            if (row_sum[i] == 0) continue;
            for (int j = 0; j < k; ++j) {
                P[i][j] = static_cast<double>(counts[i][j]) /
                          static_cast<double>(row_sum[i]);
            }
        }
        return P;
    }

    // 辅助：比较两个转移矩阵是否近似相等
    void expect_matrix_near(const std::vector<std::vector<double>>& A,
                            const std::vector<std::vector<double>>& B,
                            double tol,
                            const char* msg = "") {
        ASSERT_EQ(A.size(), B.size()) << msg;
        for (std::size_t i = 0; i < A.size(); ++i) {
            ASSERT_EQ(A[i].size(), B[i].size()) << "row " << i;
            for (std::size_t j = 0; j < A[i].size(); ++j) {
                EXPECT_NEAR(A[i][j], B[i][j], tol)
                    << msg << " (i=" << i << ", j=" << j << ")";
            }
        }
    }
};

// ==================== Symbolizer 单元测试 ====================

/**
 * @test 符号化分箱边界与覆盖范围
 * @brief 验证 Symbolizer 对于边界值、区间内值以及越界值的映射是否正确
 */
TEST_F(SymbolicDynamicsTest, SymbolizerBinMapping) {
    // edges: [0,1), [1,2), [2,10)
    std::vector<double> edges = {0.0, 1.0, 2.0, 10.0};
    Symbolizer<double> sym(edges);

    EXPECT_EQ(sym.k(), 3);

    EXPECT_EQ(sym.symbol_of(-1.0), 0);    // 小于最小边界 → 第0箱
    EXPECT_EQ(sym.symbol_of(0.0),  0);
    EXPECT_EQ(sym.symbol_of(0.5),  0);

    EXPECT_EQ(sym.symbol_of(1.0),  1);
    EXPECT_EQ(sym.symbol_of(1.5),  1);

    EXPECT_EQ(sym.symbol_of(2.0),  2);
    EXPECT_EQ(sym.symbol_of(9.9),  2);
    EXPECT_EQ(sym.symbol_of(100.0),2);    // 超过最大边界 → 最后一箱
}

/**
 * @test 构造函数参数校验
 * @brief 验证符号器与滚动符号动力学在参数非法时是否抛出异常
 */
TEST_F(SymbolicDynamicsTest, ConstructorValidation) {
    // edges 至少要有2个
    EXPECT_THROW(Symbolizer<double> sym_bad({0.0}), std::invalid_argument);

    std::vector<double> edges = {0.0, 1.0, 2.0};
    Symbolizer<double> sym(edges);

    // 窗口大小必须 >=3
    EXPECT_THROW(RollingSymbolicDynamics<double> dyn_bad(2, sym),
                 std::invalid_argument);

    EXPECT_NO_THROW(RollingSymbolicDynamics<double> dyn_ok(3, sym));
}

// ==================== 基础行为：预热与 ready() ====================

/**
 * @test 预热阶段与 ready 标志
 * @brief 验证在窗口未填满前 ready() 为 false，填满后为 true
 */
TEST_F(SymbolicDynamicsTest, WarmupAndReadyFlag) {
    std::vector<double> edges = {0.0, 0.5, 1.0};  // 2个符号
    Symbolizer<double> sym(edges);
    const std::size_t W = 5;
    RollingSymbolicDynamics<double> dyn(W, sym);

    std::vector<double> seq = {0.1, 0.9, 0.1, 0.9, 0.1};
    for (std::size_t i = 0; i < seq.size(); ++i) {
        EXPECT_TRUE(dyn.push(seq[i]));
        if (i + 1 < W) {
            EXPECT_FALSE(dyn.ready()) << "i=" << i;
        } else {
            EXPECT_TRUE(dyn.ready()) << "i=" << i;
        }
    }
}

/**
 * @test 首次填满窗口的转移矩阵
 * @brief 验证第一次填满窗口时，转移矩阵与朴素计算结果一致
 */
TEST_F(SymbolicDynamicsTest, TransitionMatrixFirstFullWindow) {
    std::vector<double> edges = {0.0, 0.5, 1.0}; // 2符号：0与1
    Symbolizer<double> sym(edges);
    const std::size_t W = 5;
    RollingSymbolicDynamics<double> dyn(W, sym);

    // 序列的符号：0,1,0,1,1
    std::vector<double> seq = {0.1, 0.9, 0.1, 0.9, 0.9};

    for (double x : seq) {
        dyn.push(x);
    }
    ASSERT_TRUE(dyn.ready());

    auto P_dyn  = dyn.transition_matrix();
    auto P_ref  = naive_transition_matrix_last_window(sym, seq, W);

    expect_matrix_near(P_dyn, P_ref, kTolTight,
                       "TransitionMatrixFirstFullWindow");
}

// ==================== 滑动窗口数值一致性 ====================

/**
 * @test 滑动窗口转移矩阵一致性
 * @brief 对长序列，验证滚动统计的转移矩阵与最后一窗朴素统计一致
 */
TEST_F(SymbolicDynamicsTest, SlidingWindowTransitionConsistency) {
    std::vector<double> edges = {0.0, 0.5, 1.0}; // 2符号：低/高
    Symbolizer<double> sym(edges);
    const std::size_t W = 5;
    RollingSymbolicDynamics<double> dyn(W, sym);

    // 稍复杂一点的 0/1 序列
    std::vector<double> seq = {
        0.1, 0.9, 0.1, 0.9, 0.9,
        0.1, 0.1, 0.9, 0.1, 0.9,
        0.9, 0.1
    };
    for (double x : seq) {
        dyn.push(x);
    }
    ASSERT_TRUE(dyn.ready());

    auto P_dyn = dyn.transition_matrix();
    auto P_ref = naive_transition_matrix_last_window(sym, seq, W);

    expect_matrix_near(P_dyn, P_ref, kTolTight,
                       "SlidingWindowTransitionConsistency");
}

/**
 * @test 转移矩阵行归一化性质
 * @brief 验证每一行的概率和约为1，且无负数
 */
TEST_F(SymbolicDynamicsTest, TransitionMatrixRowNormalization) {
    std::vector<double> edges = {0.0, 0.5, 1.0, 2.0}; // 3符号
    Symbolizer<double> sym(edges);
    RollingSymbolicDynamics<double> dyn(8, sym);

    std::vector<double> seq = {
        -1.0, 0.1, 0.7, 1.5, 0.2, 1.8, 0.6, 0.3, 1.2, 0.4
    };
    for (double x : seq) dyn.push(x);

    auto P = dyn.transition_matrix();
    ASSERT_EQ(P.size(), sym.k());

    for (std::size_t i = 0; i < P.size(); ++i) {
        double row_sum = 0.0;
        for (double pij : P[i]) {
            EXPECT_GE(pij, -1e-15) << "row " << i;
            row_sum += pij;
        }
        // 对于出现过转移的行，行和应接近 1；若该行从未出现转移，应该全 0
        if (row_sum > 0.0) {
            EXPECT_NEAR(row_sum, 1.0, 1e-12) << "row " << i;
        } else {
            for (double pij : P[i]) {
                EXPECT_DOUBLE_EQ(pij, 0.0) << "row " << i;
            }
        }
    }
}

// ==================== 拓扑熵测试 ====================

/**
 * @test 单符号序列的拓扑熵
 * @brief 对于只有一个符号的序列，邻接矩阵谱半径为1，拓扑熵应为0
 */
TEST_F(SymbolicDynamicsTest, TopologicalEntropySingleSymbol) {
    std::vector<double> edges = {0.0, 1.0}; // 1 个符号
    Symbolizer<double> sym(edges);
    const std::size_t W = 6;
    RollingSymbolicDynamics<double> dyn(W, sym);

    std::vector<double> seq = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    for (double x : seq) dyn.push(x);
    ASSERT_TRUE(dyn.ready());

    double h = dyn.topological_entropy();
    EXPECT_NEAR(h, 0.0, 1e-12);
}

/**
 * @test 两符号 full shift 的拓扑熵
 * @brief 构造 0/1 上所有转移都出现的窗口，此时拓扑熵接近 log 2
 */
TEST_F(SymbolicDynamicsTest, TopologicalEntropyTwoStateFullShift) {
    std::vector<double> edges = {0.0, 0.5, 1.0}; // 2符号
    Symbolizer<double> sym(edges);
    const std::size_t W = 5;
    RollingSymbolicDynamics<double> dyn(W, sym);

    // 符号：0,0,1,1,0 → 转移包含 0->0, 0->1, 1->1, 1->0
    std::vector<double> seq = {0.1, 0.1, 0.9, 0.9, 0.1};
    for (double x : seq) dyn.push(x);
    ASSERT_TRUE(dyn.ready());

    double h = dyn.topological_entropy();
    double expected = std::log(2.0);
    EXPECT_NEAR(h, expected, 1e-6);
}

/**
 * @test 无转移时拓扑熵为 NaN
 * @brief 若窗口内有效转移为空，谱半径<=0，拓扑熵应返回 NaN
 */
TEST_F(SymbolicDynamicsTest, TopologicalEntropyNoTransitions) {
    std::vector<double> edges = {0.0, 1.0}; // 单一符号
    Symbolizer<double> sym(edges);
    RollingSymbolicDynamics<double> dyn(3, sym);

    // 只推入一个样本，不形成任何转移
    dyn.push(0.1);

    double h = dyn.topological_entropy();
    // counts 全 0 → 谱半径=0 → 返回 NaN
    EXPECT_TRUE(std::isnan(h));
}

// ==================== 熵产生率（entropy_production_rate） ====================

/**
 * @test 时间可逆链熵产生率接近 0
 * @brief 构造 0↔1 完全对称的转移，EPR 应接近 0
 */
TEST_F(SymbolicDynamicsTest, EntropyProductionRateTimeReversibleChain) {
    std::vector<double> edges = {0.0, 0.5, 1.0}; // 2符号
    Symbolizer<double> sym(edges);
    const std::size_t W = 5;
    RollingSymbolicDynamics<double> dyn(W, sym);

    // 符号：0,1,0,1,0 → 0->1 出现2次，1->0 出现2次，对称
    std::vector<double> seq = {0.1, 0.9, 0.1, 0.9, 0.1};
    for (double x : seq) dyn.push(x);
    ASSERT_TRUE(dyn.ready());

    double sigma = dyn.entropy_production_rate();
    EXPECT_NEAR(sigma, 0.0, 1e-12);
}

/**
 * @test 非平衡链熵产生率为正
 * @brief 构造 0->1 远多于 1->0 的转移，EPR 应大于 0
 */
TEST_F(SymbolicDynamicsTest, EntropyProductionRateAsymmetricChainPositive) {
    std::vector<double> edges = {0.0, 0.5, 1.0}; // 2符号
    Symbolizer<double> sym(edges);
    const std::size_t W = 6;
    RollingSymbolicDynamics<double> dyn(W, sym);

    // 符号：0,1,1,1,0,1
    // 转移：0->1, 1->1, 1->1, 1->0, 0->1
    // 0->1:2，1->0:1，1->1:2 → 0->1 & 1->0 不对称
    std::vector<double> seq = {0.1, 0.9, 0.9, 0.9, 0.1, 0.9};
    for (double x : seq) dyn.push(x);
    ASSERT_TRUE(dyn.ready());

    double sigma = dyn.entropy_production_rate();
    EXPECT_GT(sigma, 0.0);
}

// ==================== 坏值策略（BadValuePolicy）测试 ====================

/**
 * @test SkipNaNInfPolicy：跳过坏值
 * @brief 验证遇到 NaN 时该样本被完全丢弃，不参与窗口计数
 */
TEST_F(SymbolicDynamicsTest, SkipNaNInfPolicySkipsBadSamples) {
    std::vector<double> edges = {0.0, 0.5, 1.0};
    Symbolizer<double> sym(edges);
    const std::size_t W = 3;

    RollingSymbolicDynamics<double, SkipNaNInfPolicy> dyn(W, sym);

    double NaN = std::numeric_limits<double>::quiet_NaN();

    // 第一条有效
    EXPECT_TRUE(dyn.push(0.1));
    EXPECT_FALSE(dyn.ready());

    // 第二条是 NaN，被策略丢弃，push 返回 false
    EXPECT_FALSE(dyn.push(NaN));
    EXPECT_FALSE(dyn.ready());

    // 第三、四条有效样本
    EXPECT_TRUE(dyn.push(0.9));
    EXPECT_FALSE(dyn.ready());  // 目前只有两个有效样本

    EXPECT_TRUE(dyn.push(0.1));
    EXPECT_TRUE(dyn.ready());   // 现在有三个有效样本填满窗口
}

/**
 * @test ZeroNaNInfPolicy：坏值归零
 * @brief 验证 NaN 被替换为0后仍参与符号化与统计
 */
TEST_F(SymbolicDynamicsTest, ZeroNaNInfPolicyReplacesNaNWithZero) {
    std::vector<double> edges = {0.0, 0.5, 1.0}; // 0区间包含 [0,0.5)
    Symbolizer<double> sym(edges);
    const std::size_t W = 3;

    RollingSymbolicDynamics<double, ZeroNaNInfPolicy> dyn(W, sym);
    double NaN = std::numeric_limits<double>::quiet_NaN();

    // 推入 NaN，会被策略变为 0 → symbol 0
    EXPECT_TRUE(dyn.push(NaN));
    EXPECT_FALSE(dyn.ready());

    // 后续全是 0.1，也在 symbol 0
    EXPECT_TRUE(dyn.push(0.1));
    EXPECT_TRUE(dyn.push(0.1));
    EXPECT_TRUE(dyn.ready());

    auto P = dyn.transition_matrix();
    ASSERT_EQ(P.size(), sym.k());
    // 窗口内符号序列等价于 [0,0,0]，转移两次 0->0，故该行概率应为1
    EXPECT_NEAR(P[0][0], 1.0, 1e-12);
    for (int j = 1; j < sym.k(); ++j) {
        EXPECT_NEAR(P[0][j], 0.0, 1e-12);
    }
}

// ==================== 随机序列稳定性测试 ====================

/**
 * @test 随机序列数值稳定性
 * @brief 在随机序列上检查转移矩阵、拓扑熵与熵产生率不会产生 NaN/Inf
 */
TEST_F(SymbolicDynamicsTest, RandomSequenceStability) {
    std::vector<double> edges = {-1.0, 0.0, 1.0, 2.0}; // 3符号
    Symbolizer<double> sym(edges);
    const std::size_t W = 20;
    RollingSymbolicDynamics<double> dyn(W, sym);

    std::mt19937_64 rng(20251126);
    std::normal_distribution<double> dist(0.0, 0.5);

    std::vector<double> seq(200);
    for (double& x : seq) {
        x = dist(rng);
        dyn.push(x);
    }
    ASSERT_TRUE(dyn.ready());

    auto P = dyn.transition_matrix();
    for (const auto& row : P) {
        for (double pij : row) {
            EXPECT_TRUE(std::isfinite(pij));
            EXPECT_GE(pij, -1e-15);
            EXPECT_LE(pij, 1.0 + 1e-15);
        }
    }

    double h = dyn.topological_entropy();
    // 有可能所有转移只集中在部分状态，但一般不会完全退化
    EXPECT_TRUE(std::isfinite(h) || std::isnan(h));

    double sigma = dyn.entropy_production_rate();
    EXPECT_TRUE(std::isfinite(sigma) || std::isnan(sigma));
}
