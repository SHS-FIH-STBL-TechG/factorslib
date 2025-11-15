// tests/math_tests/numeric_utils_test.cpp
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "math/numeric_utils.h"

using factorlib::math::NumericUtils;

TEST(NumericUtilsTest, SafeLogAndDivide) {
    double v1 = NumericUtils<double>::safe_log(1.0);
    EXPECT_NEAR(v1, std::log(1.0), 1e-12);

    double v0 = NumericUtils<double>::safe_log(0.0);
    double expected_min_log = std::log(std::numeric_limits<double>::min());
    EXPECT_NEAR(v0, expected_min_log, 1e-12);

    double d1 = NumericUtils<double>::safe_divide(10.0, 2.0);
    EXPECT_NEAR(d1, 5.0, 1e-12);

    double d0 = NumericUtils<double>::safe_divide(1.0, 0.0);
    EXPECT_NEAR(d0, 0.0, 1e-12);
}

TEST(NumericUtilsTest, ClampNearZeroAndLerp) {
    double c1 = NumericUtils<double>::clamp(5.0, 0.0, 10.0);
    EXPECT_NEAR(c1, 5.0, 1e-12);

    double c2 = NumericUtils<double>::clamp(-1.0, 0.0, 10.0);
    EXPECT_NEAR(c2, 0.0, 1e-12);

    double c3 = NumericUtils<double>::clamp(100.0, 0.0, 10.0);
    EXPECT_NEAR(c3, 10.0, 1e-12);

    bool near1 = NumericUtils<double>::is_near_zero(1e-10, 1e-9);
    bool near2 = NumericUtils<double>::is_near_zero(1e-3, 1e-4);
    EXPECT_TRUE(near1);
    EXPECT_FALSE(near2);

    double l = NumericUtils<double>::lerp(0.0, 10.0, 0.25);
    EXPECT_NEAR(l, 2.5, 1e-12);
}

TEST(NumericUtilsTest, Returns) {
    double lr = NumericUtils<double>::log_return(110.0, 100.0);
    EXPECT_NEAR(lr, std::log(1.1), 1e-12);

    double lr_zero = NumericUtils<double>::log_return(100.0, 0.0);
    EXPECT_NEAR(lr_zero, 0.0, 1e-12);

    double sr = NumericUtils<double>::simple_return(110.0, 100.0);
    EXPECT_NEAR(sr, 0.1, 1e-12);

    double sr_zero = NumericUtils<double>::simple_return(100.0, 0.0);
    EXPECT_NEAR(sr_zero, 0.0, 1e-12);
}
