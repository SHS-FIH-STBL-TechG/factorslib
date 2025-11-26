#pragma once

#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <limits>
#include <random>

#include "math/vector_field_analysis.h"

using namespace factorlib::math;

class VectorFieldAnalysisTest : public ::testing::Test {
protected:
    static constexpr double kTolTight = 1e-12;
    static constexpr double kTolLoose = 1e-9;

    static int idx(int i, int j, const Grid2DSize& g) {
        return i * g.ny + j;
    }

    // 生成常量场 F = (cx, cy)
    static VectorField2D make_constant_field(const Grid2DSize& g, double cx, double cy) {
        VectorField2D F;
        F.Fx.assign(g.nx * g.ny, cx);
        F.Fy.assign(g.nx * g.ny, cy);
        return F;
    }

    // 生成线性场 F(x,y) = (a * x, b * y)，x = i * hx, y = j * hy
    static VectorField2D make_linear_separable_field(const Grid2DSize& g,
                                                     double a, double b,
                                                     double hx, double hy) {
        VectorField2D F;
        F.Fx.resize(g.nx * g.ny);
        F.Fy.resize(g.nx * g.ny);
        for (int i = 0; i < g.nx; ++i) {
            double x = i * hx;
            for (int j = 0; j < g.ny; ++j) {
                double y = j * hy;
                int k = idx(i, j, g);
                F.Fx[k] = a * x;
                F.Fy[k] = b * y;
            }
        }
        return F;
    }

    // 生成刚体旋转场 F(x,y) = (-ω y, ω x)
    static VectorField2D make_rotation_field(const Grid2DSize& g,
                                             double omega,
                                             double hx, double hy) {
        VectorField2D F;
        F.Fx.resize(g.nx * g.ny);
        F.Fy.resize(g.nx * g.ny);
        for (int i = 0; i < g.nx; ++i) {
            double x = i * hx;
            for (int j = 0; j < g.ny; ++j) {
                double y = j * hy;
                int k = idx(i, j, g);
                F.Fx[k] = -omega * y;
                F.Fy[k] =  omega * x;
            }
        }
        return F;
    }
};

// ========================== 基本性质测试 ==========================

TEST_F(VectorFieldAnalysisTest, ZeroFieldHasZeroDivAndCurl) {
    Grid2DSize g{4, 5};
    VectorField2D F = make_constant_field(g, 0.0, 0.0);

    auto div = divergence(F, g);
    auto cz  = curl_z(F, g);

    ASSERT_EQ(div.size(), static_cast<std::size_t>(g.nx * g.ny));
    ASSERT_EQ(cz.size(),  static_cast<std::size_t>(g.nx * g.ny));

    for (std::size_t k = 0; k < div.size(); ++k) {
        EXPECT_NEAR(div[k], 0.0, kTolTight);
        EXPECT_NEAR(cz[k],  0.0, kTolTight);
    }
}

TEST_F(VectorFieldAnalysisTest, UniformFlowHasZeroDivAndCurl) {
    Grid2DSize g{7, 3};
    const double Ux = 2.5;
    const double Uy = -1.3;
    VectorField2D F = make_constant_field(g, Ux, Uy);

    auto div = divergence(F, g);
    auto cz  = curl_z(F, g);

    for (std::size_t k = 0; k < div.size(); ++k) {
        EXPECT_NEAR(div[k], 0.0, kTolTight);
        EXPECT_NEAR(cz[k],  0.0, kTolTight);
    }
}

// ========================== 线性场：解析可解 ==========================
// F(x,y) = (a x, b y)  =>  div F = a + b, curl_z F = 0
// 使用一般 hx, hy，边界一阶差分 + 内点中心差分，对线性函数都是精确的。

TEST_F(VectorFieldAnalysisTest, LinearSeparableFieldHasUniformDivergenceAndZeroCurl) {
    Grid2DSize g{6, 5};
    const double hx = 0.7;
    const double hy = 2.3;
    const double a  = 0.8;   // dFx/dx
    const double b  = -1.1;  // dFy/dy
    const double expected_div = a + b;

    VectorField2D F = make_linear_separable_field(g, a, b, hx, hy);

    auto div = divergence(F, g, hx, hy);
    auto cz  = curl_z(F, g, hx, hy);

    for (int i = 0; i < g.nx; ++i) {
        for (int j = 0; j < g.ny; ++j) {
            int k = idx(i, j, g);
            EXPECT_NEAR(div[k], expected_div, kTolTight) << "at (" << i << "," << j << ")";
            EXPECT_NEAR(cz[k],  0.0,          kTolTight) << "at (" << i << "," << j << ")";
        }
    }
}

// ========================== 旋转场：刚体旋转 ==========================
// F(x,y) = (-ω y, ω x)  =>  div F = 0, curl_z F = 2 ω

TEST_F(VectorFieldAnalysisTest, RotationFieldHasZeroDivergenceAndConstantCurl) {
    Grid2DSize g{5, 4};
    const double hx = 1.2;
    const double hy = 0.9;
    const double omega = 0.5;
    const double expected_curl = 2.0 * omega;

    VectorField2D F = make_rotation_field(g, omega, hx, hy);

    auto div = divergence(F, g, hx, hy);
    auto cz  = curl_z(F, g, hx, hy);

    for (int i = 0; i < g.nx; ++i) {
        for (int j = 0; j < g.ny; ++j) {
            int k = idx(i, j, g);
            EXPECT_NEAR(div[k], 0.0,          kTolTight) << "at (" << i << "," << j << ")";
            EXPECT_NEAR(cz[k],  expected_curl, kTolTight) << "at (" << i << "," << j << ")";
        }
    }
}

// ========================== 梯度场：curl 应为 0 ==========================
// φ(x,y) = 0.5 a x^2 + 0.5 b y^2
// ∇φ = (a x, b y) => 与上面的 LinearSeparable 一样，curl_z(∇φ) = 0

TEST_F(VectorFieldAnalysisTest, GradientFieldHasZeroCurl) {
    Grid2DSize g{8, 6};
    const double hx = 0.3;
    const double hy = 1.7;
    const double a  = 1.2;
    const double b  = -0.9;

    VectorField2D F;
    F.Fx.resize(g.nx * g.ny);
    F.Fy.resize(g.nx * g.ny);

    for (int i = 0; i < g.nx; ++i) {
        double x = i * hx;
        for (int j = 0; j < g.ny; ++j) {
            double y = j * hy;
            int k = idx(i, j, g);
            F.Fx[k] = a * x;
            F.Fy[k] = b * y;
        }
    }

    auto cz = curl_z(F, g, hx, hy);
    ASSERT_EQ(cz.size(), static_cast<std::size_t>(g.nx * g.ny));
    for (std::size_t k = 0; k < cz.size(); ++k) {
        EXPECT_NEAR(cz[k], 0.0, kTolTight);
    }
}

// ========================== 极小网格与非方形网格 ==========================

TEST_F(VectorFieldAnalysisTest, SingleCellGridIsHandledConsistently) {
    Grid2DSize g{1, 1};
    VectorField2D F = make_constant_field(g, 3.14, -2.71);

    auto div = divergence(F, g);
    auto cz  = curl_z(F, g);

    ASSERT_EQ(div.size(), 1u);
    ASSERT_EQ(cz.size(),  1u);

    // 差分都是自身减自身，应该为 0
    EXPECT_NEAR(div[0], 0.0, kTolTight);
    EXPECT_NEAR(cz[0],  0.0, kTolTight);
}

TEST_F(VectorFieldAnalysisTest, NonSquareGridLinearField) {
    Grid2DSize g{3, 7};
    const double hx = 0.5;
    const double hy = 2.0;
    const double a  = -0.3;
    const double b  =  0.4;
    const double expected_div = a + b;

    VectorField2D F = make_linear_separable_field(g, a, b, hx, hy);

    auto div = divergence(F, g, hx, hy);
    auto cz  = curl_z(F, g, hx, hy);

    for (int i = 0; i < g.nx; ++i) {
        for (int j = 0; j < g.ny; ++j) {
            int k = idx(i, j, g);
            EXPECT_NEAR(div[k], expected_div, kTolTight) << "at (" << i << "," << j << ")";
            EXPECT_NEAR(cz[k],  0.0,          kTolTight) << "at (" << i << "," << j << ")";
        }
    }
}

// ========================== 步长缩放一致性 ==========================
// 同一个物理场，如果 hx, hy 被整体放大/缩小，对解析导数无影响，离散实现应一致。

TEST_F(VectorFieldAnalysisTest, ScalingGridStepDoesNotChangeResultForLinearField) {
    Grid2DSize g{6, 4};
    const double a = 0.9;
    const double b = -0.2;

    // 配置 1：hx, hy 较小
    double hx1 = 0.5;
    double hy1 = 0.5;
    VectorField2D F1 = make_linear_separable_field(g, a, b, hx1, hy1);
    auto div1 = divergence(F1, g, hx1, hy1);
    auto cz1  = curl_z(F1, g, hx1, hy1);

    // 配置 2：hx, hy 放大一倍
    double hx2 = 1.0;
    double hy2 = 1.0;
    VectorField2D F2 = make_linear_separable_field(g, a, b, hx2, hy2);
    auto div2 = divergence(F2, g, hx2, hy2);
    auto cz2  = curl_z(F2, g, hx2, hy2);

    ASSERT_EQ(div1.size(), div2.size());
    ASSERT_EQ(cz1.size(),  cz2.size());

    for (std::size_t k = 0; k < div1.size(); ++k) {
        EXPECT_NEAR(div1[k], div2[k], kTolTight);
        EXPECT_NEAR(cz1[k],  cz2[k],  kTolTight);
    }
}

// ========================== 不合法输入（size mismatch） ==========================

TEST_F(VectorFieldAnalysisTest, DivergenceThrowsOnSizeMismatch) {
    Grid2DSize g{3, 3};
    VectorField2D F;
    F.Fx.assign(8, 0.0); // 少于 9
    F.Fy.assign(9, 0.0);
    EXPECT_THROW({
        auto d = divergence(F, g);
        (void)d;
    }, std::invalid_argument);
}

TEST_F(VectorFieldAnalysisTest, CurlThrowsOnSizeMismatch) {
    Grid2DSize g{4, 2};
    VectorField2D F;
    F.Fx.assign(8, 0.0);
    F.Fy.assign(5, 0.0); // 少于 8
    EXPECT_THROW({
        auto c = curl_z(F, g);
        (void)c;
    }, std::invalid_argument);
}

// ========================== 随机场健壮性（不 NaN / 不 Inf） ==========================

TEST_F(VectorFieldAnalysisTest, RandomFieldProducesFiniteOutput) {
    Grid2DSize g{10, 7};
    VectorField2D F;
    F.Fx.resize(g.nx * g.ny);
    F.Fy.resize(g.nx * g.ny);

    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist(-10.0, 10.0);

    for (int i = 0; i < g.nx * g.ny; ++i) {
        F.Fx[i] = dist(rng);
        F.Fy[i] = dist(rng);
    }

    auto div = divergence(F, g);
    auto cz  = curl_z(F, g);

    ASSERT_EQ(div.size(), static_cast<std::size_t>(g.nx * g.ny));
    ASSERT_EQ(cz.size(),  static_cast<std::size_t>(g.nx * g.ny));

    for (std::size_t k = 0; k < div.size(); ++k) {
        EXPECT_FALSE(std::isnan(div[k]));
        EXPECT_FALSE(std::isinf(div[k]));
        EXPECT_FALSE(std::isnan(cz[k]));
        EXPECT_FALSE(std::isinf(cz[k]));
    }
}
