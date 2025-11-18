#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "math/tda_rips_h1.h"

using namespace factorlib::math;

// 测试目的：在单位圆上均匀取点，小半径 ε 下图为“环”结构，β1 近似应为 1
TEST(TdaRipsH1Test, RingHasOneCycle) {
    const int N=60; // 圆周点数
    std::vector<std::vector<double>> pts(N, std::vector<double>(2));
    for (int i=0;i<N;++i){
        double th = 2*M_PI*i/N;
        pts[i][0] = std::cos(th);
        pts[i][1] = std::sin(th);
    }
    double eps = 2.2 * std::sin(M_PI/N) + 1e-6; // 连接相邻点但避免大三角
    auto G = build_sparse_graph(pts, eps);
    int b1 = beta1_approx(G);
    EXPECT_EQ(b1, 1);
}
