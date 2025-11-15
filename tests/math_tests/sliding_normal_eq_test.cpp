// tests/math_tests/sliding_normal_eq_test.cpp
#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "math/sliding_normal_eq.h"

using factorlib::math::SlidingNormalEq;
using SNE = SlidingNormalEq<double>;

TEST(SlidingNormalEqTest, BasicRegressionAndSlidingWindow) {
    const int dim    = 2;  // [1,x]
    const int window = 3;

    SNE solver(dim, window);
    SNE::Row x(dim);

    auto push_point = [&](double x_scalar, double y) {
        x(0) = 1.0;
        x(1) = x_scalar;
        solver.push(x, y);
    };

    // (0,1),(1,3),(2,5) -> y=1+2x
    push_point(0.0, 1.0);
    push_point(1.0, 3.0);
    push_point(2.0, 5.0);

    SNE::Row beta(dim);
    double RSS = 0.0;
    bool solved = solver.solve(beta, RSS);
    ASSERT_TRUE(solved);

    EXPECT_NEAR(beta(0), 1.0, 1e-10);
    EXPECT_NEAR(beta(1), 2.0, 1e-10);
    EXPECT_LE(RSS, 1e-8);

    // push (3,7) -> {1,3},{2,5},{3,7}
    push_point(3.0, 7.0);
    solved = solver.solve(beta, RSS);
    ASSERT_TRUE(solved);
    EXPECT_NEAR(beta(0), 1.0, 1e-10);
    EXPECT_NEAR(beta(1), 2.0, 1e-10);
    EXPECT_LE(RSS, 1e-8);
}

TEST(SlidingNormalEqTest, ResetAndSolveOnEmpty) {
    SNE solver(2, 3);
    SNE::Row x(2);
    x << 1.0, 0.0;

    solver.push(x, 1.0);
    solver.push(x, 1.0);
    EXPECT_EQ(solver.size(), 2);

    solver.reset();
    EXPECT_EQ(solver.size(), 0);

    SNE::Row beta(2);
    double RSS = 0.0;
    bool solved = solver.solve(beta, RSS);
    EXPECT_FALSE(solved);
}
