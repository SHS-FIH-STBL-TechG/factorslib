// tests/math_tests/incremental_rank_test.cpp
#include <vector>
#include <cmath>

#include <gtest/gtest.h>

#include "math/incremental_rank.h"

using factorlib::math::IncrementalRankCalculator;

TEST(IncrementalRankTest, BasicWindowAndMedianRank) {
    const std::size_t window_size = 3;

    IncrementalRankCalculator<double> calc;

    calc.push(10.0, window_size);
    calc.push(20.0, window_size);
    calc.push(30.0, window_size);

    EXPECT_TRUE(calc.is_window_full(window_size));
    EXPECT_EQ(calc.size(), 3u);

    std::vector<double> sorted = calc.get_sorted_data();
    ASSERT_EQ(sorted.size(), 3u);
    EXPECT_NEAR(sorted[0], 10.0, 1e-12);
    EXPECT_NEAR(sorted[1], 20.0, 1e-12);
    EXPECT_NEAR(sorted[2], 30.0, 1e-12);

    // [10,20,30], N=3
    // 10 -> (0+0.5)/3 = 1/6
    double r10 = calc.median_rank(10.0);
    EXPECT_NEAR(r10, 0.5 / 3.0, 1e-12);

    // 25 -> lower_bound index=2 -> (2+0.5)/3 = 2.5/3
    double r25 = calc.median_rank(25.0);
    EXPECT_NEAR(r25, 2.5 / 3.0, 1e-12);

    // push 40 -> window [20,30,40]
    calc.push(40.0, window_size);
    EXPECT_TRUE(calc.is_window_full(window_size));
    EXPECT_EQ(calc.size(), 3u);

    sorted = calc.get_sorted_data();
    ASSERT_EQ(sorted.size(), 3u);
    EXPECT_NEAR(sorted[0], 20.0, 1e-12);
    EXPECT_NEAR(sorted[1], 30.0, 1e-12);
    EXPECT_NEAR(sorted[2], 40.0, 1e-12);
}
