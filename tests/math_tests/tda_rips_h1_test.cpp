#pragma once

#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <functional>   // 确保 tda_rips_h1.h 里的 std::function 可用

#include "math/tda_rips_h1.h"

using namespace factorlib::math;

// 方便一点的别名
using Point  = std::vector<double>;
using Points = std::vector<Point>;

// ==================== 测试夹具 ====================

class TdaRipsH1Test : public ::testing::Test {
protected:
    static constexpr double kTolTight = 1e-12;
    static constexpr double kTolLoose = 1e-6;

    // 工具：构造一维点
    Points make_line_points_1d(int n, double step = 1.0) {
        Points pts;
        pts.reserve(n);
        for (int i = 0; i < n; ++i) {
            pts.push_back(Point{ step * i });
        }
        return pts;
    }

    // 工具：构造正方形四个顶点（2D）
    Points make_unit_square() {
        return {
            {0.0, 0.0}, // 0
            {1.0, 0.0}, // 1
            {1.0, 1.0}, // 2
            {0.0, 1.0}  // 3
        };
    }

    // 工具：构造单位圆上均匀采样点
    Points make_circle_points(int n) {
        Points pts;
        pts.reserve(n);
        for (int i = 0; i < n; ++i) {
            double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
            pts.push_back(Point{ std::cos(theta), std::sin(theta) });
        }
        return pts;
    }

    // 工具：计算两点欧氏距离
    double dist(const Point& a, const Point& b) {
        double s = 0.0;
        for (std::size_t i = 0; i < a.size(); ++i) {
            double d = a[i] - b[i];
            s += d * d;
        }
        return std::sqrt(s);
    }
};

// ==================== GridDownSampler 测试 ====================

/**
 * @test 空输入与 Nmax = 0 的退化行为
 * @brief 验证空点集与 0 限制下不会崩溃，并返回空结果
 */
TEST_F(TdaRipsH1Test, GridDownSamplerEmptyAndZero) {
    Points empty;
    auto out1 = GridDownSampler::downsample(empty, 1.0, 0);
    EXPECT_TRUE(out1.empty());

    auto out2 = GridDownSampler::downsample(empty, 0.5, 100);
    EXPECT_TRUE(out2.empty());
}

/**
 * @test 同一网格保留首个点
 * @brief 多个点落在同一 cell 时，仅保留第一个
 */
TEST_F(TdaRipsH1Test, GridDownSamplerSameCellKeepsFirst) {
    Points pts = {
        {0.1, 0.1}, // cell (0,0)
        {0.9, 0.9}, // cell (0,0)
        {1.1, 0.1}, // cell (1,0)
        {1.2, 1.7}  // cell (1,1)
    };

    double cell = 1.0;
    auto out = GridDownSampler::downsample(pts, cell);

    // 4 个点分布在 3 个 cell，因此应保留 3 个代表点
    ASSERT_EQ(out.size(), 3u);

    // 第一个 cell(0,0) 应该保留 pts[0]
    EXPECT_DOUBLE_EQ(out[0][0], pts[0][0]);
    EXPECT_DOUBLE_EQ(out[0][1], pts[0][1]);

    // 剩下两个是另两个 cell 的代表点（顺序由输入顺序决定）
    // 不强行检查顺序，只检查它们出现在原集合中
    auto is_in_original = [&](const Point& p) {
        for (const auto& q : pts) {
            if (dist(p, q) < kTolTight) return true;
        }
        return false;
    };
    EXPECT_TRUE(is_in_original(out[1]));
    EXPECT_TRUE(is_in_original(out[2]));
}

/**
 * @test Nmax 限制行为
 * @brief 不同 cell 但 Nmax 小于 cell 数时，应截断输出
 */
TEST_F(TdaRipsH1Test, GridDownSamplerNmaxLimit) {
    // 1D 上 10 个不同行的点，每个点落在不同 cell
    Points pts = make_line_points_1d(10, 1.0); // 0,1,...,9
    double cell = 0.5;
    std::size_t Nmax = 5;

    auto out = GridDownSampler::downsample(pts, cell, Nmax);
    ASSERT_EQ(out.size(), Nmax);

    // 保证保留的是前 Nmax 个点
    for (std::size_t i = 0; i < Nmax; ++i) {
        ASSERT_EQ(out[i].size(), 1u);
        EXPECT_DOUBLE_EQ(out[i][0], pts[i][0]);
    }
}

// ==================== build_sparse_graph 基础图结构测试 ====================

/**
 * @test 一维直线点构造稀疏图
 * @brief eps 选择为仅连接相邻点，验证边与度数
 */
TEST_F(TdaRipsH1Test, BuildSparseGraphLine1D) {
    Points pts = make_line_points_1d(4, 1.0); // 0,1,2,3
    double eps = 1.01;                         // 只连接相邻间隔为 1 的点

    auto G = build_sparse_graph(pts, eps);

    ASSERT_EQ(G.V, pts.size());
    // 预期边：(0-1),(1-2),(2-3)
    EXPECT_EQ(G.edges.size(), 3u);

    // 检查每个点的度数
    ASSERT_EQ(G.adj.size(), G.V);
    EXPECT_EQ(G.adj[0].size(), 1u);
    EXPECT_EQ(G.adj[1].size(), 2u);
    EXPECT_EQ(G.adj[2].size(), 2u);
    EXPECT_EQ(G.adj[3].size(), 1u);
}

/**
 * @test 正方形点集构造无对角线的图
 * @brief eps 选择为只连接边长 1 的边，检查邻接结构大致合理
 */
TEST_F(TdaRipsH1Test, BuildSparseGraphSquareNoDiagonals) {
    Points pts = make_unit_square();
    double eps = 1.01; // 只连接距离 1 的边，不应连接对角线 sqrt(2)

    auto G = build_sparse_graph(pts, eps);

    ASSERT_EQ(G.V, pts.size());
    // 四条边组成一个环
    EXPECT_EQ(G.edges.size(), 4u);

    // 每个点度数应该为 2
    ASSERT_EQ(G.adj.size(), G.V);
    for (std::size_t i = 0; i < G.V; ++i) {
        EXPECT_EQ(G.adj[i].size(), 2u) << "vertex " << i;
    }
}

/**
 * @test 空点集构图
 * @brief 验证空输入不会崩溃，图结构为空
 */
TEST_F(TdaRipsH1Test, BuildSparseGraphEmpty) {
    Points pts;
    double eps = 1.0;

    auto G = build_sparse_graph(pts, eps);
    EXPECT_EQ(G.V, 0u);
    EXPECT_TRUE(G.adj.empty());
    EXPECT_TRUE(G.edges.empty());
}

// ==================== beta1_approx 拓扑结构测试 ====================

/**
 * @test β1 在线段图上的行为
 * @brief 对于一条线型连通图，应有 β1=0
 */
TEST_F(TdaRipsH1Test, Beta1OnLineGraphIsZero) {
    Points pts = make_line_points_1d(4, 1.0); // 0-1-2-3
    double eps = 1.01;

    auto G = build_sparse_graph(pts, eps);
    int beta1 = beta1_approx(G);

    EXPECT_EQ(beta1, 0);
}

/**
 * @test β1 在正方形环图上的行为
 * @brief 无对角线的 4 点环，β1 应为 1
 */
TEST_F(TdaRipsH1Test, Beta1OnSquareCycleIsOne) {
    Points pts = make_unit_square();
    double eps = 1.01; // 只连边，不连对角线

    auto G = build_sparse_graph(pts, eps);
    int beta1 = beta1_approx(G);

    // 期望：E=4, V=4, C=1, T=0 → β1 = 4 - 4 + 1 - 0 = 1
    EXPECT_EQ(beta1, 1);
}

/**
 * @test β1 在三角形完整图上的行为
 * @brief K3 等价于填充三角形，其一维同调应近似为 0
 */
TEST_F(TdaRipsH1Test, Beta1OnCompleteTriangleIsZero) {
    Points pts = {
        {0.0, 0.0},
        {1.0, 0.0},
        {0.5, std::sqrt(3.0) / 2.0}
    };
    double eps = 2.0; // 远大于任意边长，构造 K3 完全图

    auto G = build_sparse_graph(pts, eps);
    int beta1 = beta1_approx(G);

    // 完整三角形被视为“填充”的 2-单形，β1 ≈ 0
    EXPECT_EQ(beta1, 0);
}

/**
 * @test β1 在单位圆采样点上的行为
 * @brief 小 eps 只连接相邻点，理想上得到一个单圈，β1 应为 1
 */
TEST_F(TdaRipsH1Test, Beta1OnCircleIsApproximatelyOne) {
    const int N = 40;
    Points pts = make_circle_points(N);

    // 邻近点间弦长约为 2 sin(pi/N) ~ 2*pi/N ~ 0.157
    // eps 选 0.3，仅连接最近邻，不连接更远点
    double eps = 0.3;

    auto G = build_sparse_graph(pts, eps);
    int beta1 = beta1_approx(G);

    // 对“稀疏 Rips 1-skeleton”来说，这个形状应表现为一个主圈
    EXPECT_EQ(beta1, 1);
}

/**
 * @test β1 空图上的行为
 * @brief 空图 β1 应为 0
 */
TEST_F(TdaRipsH1Test, Beta1OnEmptyGraphIsZero) {
    SparseGraph G;
    G.V = 0;
    int beta1 = beta1_approx(G);
    EXPECT_EQ(beta1, 0);
}

/**
 * @test β1 非负性
 * @brief 实现内部对负值做了截断，任意稀疏图输出都应 >=0
 */
TEST_F(TdaRipsH1Test, Beta1AlwaysNonNegative) {
    // 构造一个稍复杂的图：两条小链 + 一条环
    Points pts;
    // 链 1: 3 点
    auto chain1 = make_line_points_1d(3, 1.0); // 0,1,2
    pts.insert(pts.end(), chain1.begin(), chain1.end());
    // 链 2: 3 点，平移 10
    auto chain2 = make_line_points_1d(3, 1.0);
    for (auto& p : chain2) p[0] += 10.0;       // 10,11,12
    pts.insert(pts.end(), chain2.begin(), chain2.end());
    // 环: 正方形，平移 20
    auto square = make_unit_square();
    for (auto& p : square) p[0] += 20.0;
    pts.insert(pts.end(), square.begin(), square.end());

    double eps_line   = 1.01;
    double eps_square = 1.01;

    // 对于不同 eps 值分别构图并求 β1，验证结果非负
    auto G1 = build_sparse_graph(pts, eps_line);
    int b1 = beta1_approx(G1);
    EXPECT_GE(b1, 0);

    auto G2 = build_sparse_graph(pts, eps_square);
    int b2 = beta1_approx(G2);
    EXPECT_GE(b2, 0);
}
