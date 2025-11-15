// tests/math_tests/distributions_test.cpp
#include <vector>
#include <deque>
#include <cmath>

#include <gtest/gtest.h>

#include "math/distributions.h"

using factorlib::math::Distributions;

// ===================== empirical_inverse_cdf =====================

TEST(DistributionsTest, EmpiricalInverseCdfVector) {
    // data={3,1,7,9}, sorted={1,3,7,9}, N=4
    std::vector<double> data{3.0, 1.0, 7.0, 9.0};

    double p0  = Distributions::empirical_inverse_cdf(data, 0.0);
    double p1  = Distributions::empirical_inverse_cdf(data, 1.0);
    double p05 = Distributions::empirical_inverse_cdf(data, 0.5);
    double p33 = Distributions::empirical_inverse_cdf(data, 1.0 / 3.0);

    EXPECT_NEAR(p0,  1.0, 1e-12);
    EXPECT_NEAR(p1,  9.0, 1e-12);
    EXPECT_NEAR(p05, 5.0, 1e-12);
    EXPECT_NEAR(p33, 3.0, 1e-12);
}

TEST(DistributionsTest, EmpiricalInverseCdfDeque) {
    std::deque<int> data{3, 1, 7, 9};

    double p05 = Distributions::empirical_inverse_cdf(data, 0.5);
    EXPECT_NEAR(p05, 5.0, 1e-12);
}

// ===================== normal_cdf（新增数学方法） =====================

TEST(DistributionsTest, NormalCdfBasicValues) {
    EXPECT_NEAR(Distributions::normal_cdf(0.0), 0.5, 1e-15);

    EXPECT_NEAR(Distributions::normal_cdf(1.0),
                0.8413447460685429, 1e-15);

    EXPECT_NEAR(Distributions::normal_cdf(-1.0),
                0.15865525393145707, 1e-15);

    EXPECT_NEAR(Distributions::normal_cdf(2.0),
                0.9772498680518208, 1e-15);

    EXPECT_NEAR(Distributions::normal_cdf(-2.0),
                0.02275013194817922, 1e-15);
}

TEST(DistributionsTest, NormalCdfTemplateFloatAndLongDouble) {
    float z_f = 1.0f;
    float F_f = Distributions::normal_cdf(z_f);
    EXPECT_NEAR(F_f, 0.8413447f, 1e-6f);  // float 精度放宽一些

    long double z_ld = static_cast<long double>(-1.0);
    long double F_ld = Distributions::normal_cdf(z_ld);
    long double expected_ld = static_cast<long double>(0.15865525393145707);
    EXPECT_NEAR(static_cast<double>(F_ld),
                static_cast<double>(expected_ld),
                1e-15);
}
