// tests/math_tests/linear_algebra_test.cpp
#include <vector>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "math/linear_algebra.h"

using factorlib::math::LinearAlgebra;

TEST(LinearAlgebraTest, CovarianceAndRegularization) {
    std::vector<std::vector<double>> data{
            {1.0, 2.0},
            {3.0, 4.0},
            {5.0, 6.0}
    };

    Eigen::MatrixXd cov = LinearAlgebra<double>::covariance_matrix(data);

    ASSERT_EQ(cov.rows(), 2);
    ASSERT_EQ(cov.cols(), 2);

    EXPECT_NEAR(cov(0,0), 4.0, 1e-12);
    EXPECT_NEAR(cov(0,1), 4.0, 1e-12);
    EXPECT_NEAR(cov(1,0), 4.0, 1e-12);
    EXPECT_NEAR(cov(1,1), 4.0, 1e-12);

    double reg = 1e-3;
    Eigen::MatrixXd cov_reg =
        LinearAlgebra<double>::regularize_covariance(cov, reg);

    EXPECT_NEAR(cov_reg(0,0), 4.0 + reg, 1e-12);
    EXPECT_NEAR(cov_reg(1,1), 4.0 + reg, 1e-12);
    EXPECT_NEAR(cov_reg(0,1), 4.0,       1e-12);
    EXPECT_NEAR(cov_reg(1,0), 4.0,       1e-12);

    double cond = LinearAlgebra<double>::condition_number(cov);
    if (std::isfinite(cond)) {
        EXPECT_GT(cond, 1e6) << "condition number should be huge for singular matrix";
    }
}

TEST(LinearAlgebraTest, ConditionalExpectation) {
    Eigen::VectorXd mean(2);
    mean << 0.0, 0.0;

    Eigen::MatrixXd cov(2,2);
    cov << 1.0, 0.5,
           0.5, 2.0;

    Eigen::VectorXd cond_vals(1);
    cond_vals << 1.0;

    double ce =
        LinearAlgebra<double>::conditional_expectation(mean, cov, cond_vals, 1);

    EXPECT_NEAR(ce, 0.5, 1e-12) << "E[X2|X1=1] should be 0.5";
}
