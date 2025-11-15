// tests/math_tests/statistics_test.cpp
#include <vector>
#include <cmath>

#include <gtest/gtest.h>

#include "math/statistics.h"

using factorlib::math::Statistics;

TEST(StatisticsTest, MeanAndStddev) {
    std::vector<double> data{1.0, 2.0, 3.0, 4.0};

    double m = Statistics<double>::mean(data);
    EXPECT_NEAR(m, 2.5, 1e-12);

    double expected_std = std::sqrt(5.0 / 3.0);
    double s = Statistics<double>::stddev(data);
    EXPECT_NEAR(s, expected_std, 1e-12);
}

TEST(StatisticsTest, QuantileAndMedian) {
    std::vector<double> data{1.0, 2.0, 3.0, 4.0};

    double q0  = Statistics<double>::quantile(data, 0.0);
    double q05 = Statistics<double>::quantile(data, 0.5);
    double q1  = Statistics<double>::quantile(data, 1.0);
    double med = Statistics<double>::median(data);

    EXPECT_NEAR(q0,  1.0, 1e-12);
    EXPECT_NEAR(q05, 2.5, 1e-12);
    EXPECT_NEAR(q1,  4.0, 1e-12);
    EXPECT_NEAR(med, 2.5, 1e-12);
}

TEST(StatisticsTest, MedianRankCovCorrRolling) {
    // median_rank
    std::vector<double> data{1.0, 3.0, 5.0, 7.0};
    double r1 = Statistics<double>::median_rank(data, 1.0);
    double r4 = Statistics<double>::median_rank(data, 4.0);
    EXPECT_NEAR(r1, 0.125, 1e-12);
    EXPECT_NEAR(r4, 0.625, 1e-12);

    // covariance & correlation: x=[1,2,3], y=[2,4,6]
    std::vector<double> x{1.0, 2.0, 3.0};
    std::vector<double> y{2.0, 4.0, 6.0};

    double cov = Statistics<double>::covariance(x, y);
    double corr = Statistics<double>::correlation(x, y);

    EXPECT_NEAR(cov,  2.0, 1e-12);
    EXPECT_NEAR(corr, 1.0, 1e-12);

    // rolling_mean: [1,2,3,4,5] window=3 -> [2,3,4]
    std::vector<double> data2{1,2,3,4,5};
    std::vector<double> rm = Statistics<double>::rolling_mean(data2, 3);

    ASSERT_EQ(rm.size(), 3u);
    EXPECT_NEAR(rm[0], 2.0, 1e-12);
    EXPECT_NEAR(rm[1], 3.0, 1e-12);
    EXPECT_NEAR(rm[2], 4.0, 1e-12);
}
