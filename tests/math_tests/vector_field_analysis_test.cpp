#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "math/vector_field_analysis.h"

using namespace factorlib::math;

// 测试用例 1：在 Fx=x, Fy=0 的简单双向量场上，散度应≈1、旋度≈0（边界有限差分容差放宽）
// 入参：网格大小 5x5，Fx(i,j)=i, Fy(i,j)=0。
// 期望：divergence 每个网格点≈1，curl_z≈0。
TEST(VectorFieldAnalysisTest, DivergenceAndCurlSimpleField) {
    Grid2DSize g{5,5};
    VectorField2D F;
    F.Fx.resize(g.nx*g.ny);
    F.Fy.resize(g.nx*g.ny);
    auto idx=[&](int i,int j){ return i*g.ny+j; };
    for (int i=0;i<g.nx;++i) for (int j=0;j<g.ny;++j) {
        double x=i, y=j;
        F.Fx[idx(i,j)] = x; // Fx = x -> dFx/dx = 1
        F.Fy[idx(i,j)] = 0; // Fy = 0 -> dFy/dy = 0
    }
    auto div = divergence(F, g);
    auto cz  = curl_z(F, g);
    for (auto v : div) EXPECT_NEAR(v, 1.0, 0.25);
    for (auto v : cz)  EXPECT_NEAR(v, 0.0, 0.25);
}

// 测试用例 2：在旋转向量场 Fx=-y, Fy=x 上，散度≈0，旋度≈2
// 入参：网格大小 5x5，Fx(i,j)=-j, Fy(i,j)=i。
// 期望：divergence 接近 0；curl_z=dFy/dx - dFx/dy≈1-(-1)=2。
TEST(VectorFieldAnalysisTest, DivergenceZeroCurlTwoForRotation) {
    Grid2DSize g{5,5};
    VectorField2D F;
    F.Fx.resize(g.nx*g.ny);
    F.Fy.resize(g.nx*g.ny);
    auto idx=[&](int i,int j){ return i*g.ny+j; };
    for (int i=0;i<g.nx;++i) for (int j=0;j<g.ny;++j) {
        double x=i, y=j;
        F.Fx[idx(i,j)] = -y; // Fx = -y
        F.Fy[idx(i,j)] =  x; // Fy =  x
    }
    auto div = divergence(F, g);
    auto cz  = curl_z(F, g);
    for (auto v : div) EXPECT_NEAR(v, 0.0, 0.5);  // 允许更大的边界误差
    for (auto v : cz)  EXPECT_NEAR(v, 2.0, 0.5);
}
