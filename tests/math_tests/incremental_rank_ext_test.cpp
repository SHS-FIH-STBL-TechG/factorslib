// tests/math_tests/incremental_rank_with_aux_test.cpp
#include <vector>
#include <cmath>

#include <gtest/gtest.h>

#include "math/incremental_rank_ext.h"

using factorlib::math::IncrementalRankWithAux;

// ============================================================================
// 基础窗口与排序视图测试
// ============================================================================
TEST(IncrementalRankWithAuxTest, BasicWindowAndSortedView) {
    const std::size_t window_size = 3;

    // 主序列 = [10, 20, 30]
    // AUX 序列 = [1, 2, 3]
    IncrementalRankWithAux<double, double> calc;

    calc.push(10.0, 1.0, window_size);
    calc.push(20.0, 2.0, window_size);
    calc.push(30.0, 3.0, window_size);

    EXPECT_TRUE(calc.is_window_full(window_size));
    EXPECT_EQ(calc.size(), 3u);

    // 排序视图应为 [10,20,30]
    auto sorted = calc.get_sorted_data();
    ASSERT_EQ(sorted.size(), 3u);
    EXPECT_NEAR(sorted[0], 10.0, 1e-12);
    EXPECT_NEAR(sorted[1], 20.0, 1e-12);
    EXPECT_NEAR(sorted[2], 30.0, 1e-12);

    // median_rank 行为与基类一致
    EXPECT_NEAR(calc.median_rank(10.0), (0.5)/3.0, 1e-12);  // rank=0
    EXPECT_NEAR(calc.median_rank(25.0), (2.5)/3.0, 1e-12);  // rank=2
}

// ============================================================================
// flag（0/1）增量更新 & 时间/排序视图
// ============================================================================
TEST(IncrementalRankWithAuxTest, ThresholdFlagsIncremental) {
    const std::size_t window_size = 4;
    IncrementalRankWithAux<double, double> calc;

    // 主序列时间顺序 [5,10,2,6]
    // AUX 序列时间顺序 [1,2,3,4]
    calc.push(5.0, 1.0, window_size);
    calc.push(10.0,2.0, window_size);
    calc.push(2.0, 3.0, window_size);
    calc.push(6.0, 4.0, window_size);

    // 排序视图 [2,5,6,10]
    auto sorted = calc.get_sorted_data();
    ASSERT_EQ(sorted.size(), 4u);
    EXPECT_NEAR(sorted[0], 2.0, 1e-12);
    EXPECT_NEAR(sorted[1], 5.0, 1e-12);
    EXPECT_NEAR(sorted[2], 6.0, 1e-12);
    EXPECT_NEAR(sorted[3],10.0, 1e-12);

    double th = (5.0 + 10.0 + 2.0 + 6.0) / 4.0; // 5.75

    calc.update_binary_flags(th);

    // sorted flag = [0,0,1,1]
    auto flags_sorted = calc.get_binary_flags_sorted_order();
    ASSERT_EQ(flags_sorted.size(), 4u);
    EXPECT_EQ(flags_sorted[0], 0);
    EXPECT_EQ(flags_sorted[1], 0);
    EXPECT_EQ(flags_sorted[2], 1);
    EXPECT_EQ(flags_sorted[3], 1);

    // time flag = [0,1,0,1]
    auto flags_time = calc.get_binary_flags_time_order();
    ASSERT_EQ(flags_time.size(), 4u);
    EXPECT_EQ(flags_time[0], 0); // 5
    EXPECT_EQ(flags_time[1], 1); // 10
    EXPECT_EQ(flags_time[2], 0); // 2
    EXPECT_EQ(flags_time[3], 1); // 6
}

// ============================================================================
// AUX 增量均值基础行为测试
// ============================================================================
TEST(IncrementalRankWithAuxTest, AuxFlaggedMeanBasic) {
    const std::size_t window_size = 4;
    IncrementalRankWithAux<double, double> calc;

    // 主序列 [5,10,2,6]
    // AUX    [1, 2,3,4]
    calc.push(5.0, 1.0, window_size);
    calc.push(10.0,2.0, window_size);
    calc.push(2.0, 3.0, window_size);
    calc.push(6.0, 4.0, window_size);

    double th = 5.75;
    calc.update_binary_flags(th);

    // flag=1 的主值 = {6,10}
    double main_mean_manual = (6.0 + 10.0) / 2.0;

    EXPECT_NEAR(calc.flagged_mean(), main_mean_manual, 1e-12);

    // 对应 AUX = {4,2}
    double aux_manual = (4.0 + 2.0) / 2.0;
    EXPECT_NEAR(calc.aux_flagged_mean(), aux_manual, 1e-12);
}

// ============================================================================
// 综合测试：窗口滑动 + flag 翻转 + AUX 增量维护
// ============================================================================
TEST(IncrementalRankWithAuxTest, AuxFlaggedMeanIncrementalUpdate) {
    const std::size_t window_size = 3;

    IncrementalRankWithAux<double,double> calc;

    // 初始窗口主序列 [10,20,5]
    // AUX 序列       [1, 2, 3]
    calc.push(10.0,1.0, window_size);
    calc.push(20.0,2.0, window_size);
    calc.push(5.0, 3.0, window_size);

    // 阈值=12 → flag=[0,1,0], 主均值=20，AUX均值=2
    calc.update_binary_flags(12.0);
    EXPECT_NEAR(calc.flagged_mean(),      20.0, 1e-12);
    EXPECT_NEAR(calc.aux_flagged_mean(),  2.0,  1e-12);

    // push 30 → 新窗口[20,5,30] 删除10
    // AUX→    [2, 3, 4]
    calc.push(30.0,4.0, window_size);

    // 阈值=15 → flag=[1,0,1], 主均值=(20+30)/2，AUX均值=(2+4)/2
    calc.update_binary_flags(15.0);
    EXPECT_NEAR(calc.flagged_mean(),      (20.0+30.0)/2.0, 1e-12);
    EXPECT_NEAR(calc.aux_flagged_mean(),  (2.0 +4.0 )/2.0, 1e-12);

    // push 3 → 新窗口[5,30,3] 删除20
    // AUX→    [3, 4, 6]
    calc.push(3.0,6.0, window_size);

    // 阈值=10 → flag=[0,1,0], 主均值=30，AUX均值=4
    calc.update_binary_flags(10.0);
    EXPECT_NEAR(calc.flagged_mean(),     30.0, 1e-12);
    EXPECT_NEAR(calc.aux_flagged_mean(), 4.0,  1e-12);
}
