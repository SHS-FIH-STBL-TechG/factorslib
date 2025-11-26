#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <memory>

#include "math/persistent_homology_h0.h"

using namespace factorlib::math;

// ==========================================
// 基础功能测试夹具
// ==========================================

/**
 * @brief 测试夹具：基础功能测试
 * 使用 double 类型，默认坏值策略 (SkipNaNInfPolicy)
 */
class SlidingWindowH0Test : public ::testing::Test {
protected:
    std::unique_ptr<SlidingWindowPersistenceH0<double>> analyzer;

    void SetUp() override {
        analyzer = std::make_unique<SlidingWindowPersistenceH0<double>>(5);
    }

    void TearDown() override {
        analyzer.reset();
    }
};

// ==========================================
// 核心逻辑与滑动窗口测试
// ==========================================

/**
 * @brief 测试用例 1: 基础增量更新
 * 验证点添加、排序及间隙(Gap)计算的正确性
 * 数学原理：Gap = |b - a| / 2，反映连通分量合并的阈值
 */
TEST_F(SlidingWindowH0Test, BasicIncrements) {
    // 1. 添加第一个点
    analyzer->push(10.0);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.0);
    EXPECT_EQ(analyzer->size(), 1);

    // 2. 添加第二个点: {10, 20}, Gap = 5.0
    analyzer->push(20.0);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 5.0);
    EXPECT_EQ(analyzer->size(), 2);

    // 3. 插入中间点: {10, 15, 20}
    // Gaps: (15-10)/2=2.5, (20-15)/2=2.5. Mean = 2.5
    analyzer->push(15.0);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 2.5);
    EXPECT_EQ(analyzer->size(), 3);
}

/**
 * @brief 测试用例 2: 滑动窗口 FIFO 逻辑
 * 验证窗口满后的移除操作是否正确更新统计信息
 * 重点测试时间窗口与有序集合的同步更新
 */
TEST_F(SlidingWindowH0Test, SlidingWindowLogic) {
    analyzer->reset(3); // 重置窗口大小为 3

    // 填满窗口: [10, 20, 30] -> Sorted: {10, 20, 30}
    // Gaps: 5.0, 5.0 -> Mean: 5.0
    analyzer->push_points({10.0, 20.0, 30.0});
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 5.0);
    EXPECT_TRUE(analyzer->ready());

    // 推入 40: 窗口变成 [20, 30, 40] (10 被移除)
    // Sorted: {20, 30, 40}. Gaps: 5.0, 5.0
    analyzer->push(40.0);
    EXPECT_EQ(analyzer->size(), 3);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 5.0);

    // 推入 25: 窗口变成 [30, 40, 25] (20 被移除)
    // Sorted: {25, 30, 40}.
    // Gaps: (30-25)/2=2.5, (40-30)/2=5.0. Total=7.5, Count=2, Mean=3.75
    analyzer->push(25.0);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 3.75);
}

/**
 * @brief 测试用例 3: 阈值过滤 (Eps Max)
 * 验证大于阈值的间隙不参与统计
 * 测试动态阈值更新的实时性
 */
TEST_F(SlidingWindowH0Test, EpsilonThreshold) {
    analyzer->update_eps_max(10.0); // 只有 gap <= 10 才算有效

    // {0, 100, 102}
    // Gap1: (100-0)/2 = 50 (>10, 忽略)
    // Gap2: (102-100)/2 = 1 (<=10, 计入)
    analyzer->push(0.0);
    analyzer->push(100.0);
    analyzer->push(102.0);

    EXPECT_EQ(analyzer->valid_gap_count(), 1);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 1.0);

    // 动态放宽阈值
    analyzer->update_eps_max(60.0);
    // 现在 50 也计入了。 Total=51, Count=2, Mean=25.5
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 25.5);
    EXPECT_EQ(analyzer->valid_gap_count(), 2);
}

/**
 * @brief 测试用例 4: 静态计算的一致性验证
 * 使用随机数据对比增量算法与静态重算的结果
 * 验证增量计算的数学正确性
 */
TEST_F(SlidingWindowH0Test, ConsistencyWithStatic) {
    size_t window_size = 50;
    analyzer->reset(window_size);
    std::mt19937 gen(42);
    std::uniform_real_distribution<> dis(-1000.0, 1000.0);
    std::vector<double> history;

    for (int i = 0; i < 200; ++i) {
        double val = dis(gen);
        history.push_back(val);
        analyzer->push(val);

        if (history.size() >= 2) {
            // 准备静态数据
            std::vector<double> current_window;
            size_t start = (history.size() > window_size) ? history.size() - window_size : 0;
            for (size_t j = start; j < history.size(); ++j) current_window.push_back(history[j]);

            // 注意：类内部使用 std::set 自动去重，compute_static 需要手动去重以匹配行为
            std::sort(current_window.begin(), current_window.end());
            current_window.erase(std::unique(current_window.begin(), current_window.end()), current_window.end());

            double static_mean = SlidingWindowPersistenceH0<double>::compute_static(
                current_window, std::numeric_limits<double>::max()
            );
            double inc_mean = analyzer->get_mean_persistence();

            ASSERT_NEAR(inc_mean, static_mean, 1e-9)
                << "Mismatch at step " << i << " val=" << val;
        }
    }
}

// ==========================================
// 坏值处理策略测试 (BadValuePolicy)
// ==========================================

/**
 * @brief 策略测试 1: SkipNaNInfPolicy (默认)
 * 遇到 NaN/Inf 应返回 false，且不影响窗口内的统计
 * 验证坏值被正确跳过，不影响正常数据
 */
TEST(PolicyTest, SkipPolicyBehavior) {
    SlidingWindowPersistenceH0<double, SkipNaNInfPolicy> analyzer(5);

    analyzer.push(10.0);
    size_t size_before = analyzer.size();
    int processed_before = analyzer.total_processed_points();

    // 尝试推入 NaN
    bool ret = analyzer.push(std::numeric_limits<double>::quiet_NaN());

    EXPECT_FALSE(ret) << "Skip策略下应返回 false";
    EXPECT_EQ(analyzer.size(), size_before) << "有效窗口大小不应增加";
    EXPECT_EQ(analyzer.total_processed_points(), processed_before + 1) << "处理总数应增加";

    // 验证正常数据不受影响
    analyzer.push(20.0);
    EXPECT_DOUBLE_EQ(analyzer.get_mean_persistence(), 5.0);
}

/**
 * @brief 策略测试 2: ZeroNaNInfPolicy
 * 遇到 NaN/Inf 应替换为 0，返回 true，并参与计算
 * 验证坏值被替换为0后的正确计算
 */
TEST(PolicyTest, ZeroPolicyBehavior) {
    SlidingWindowPersistenceH0<double, ZeroNaNInfPolicy> analyzer(5);

    analyzer.push(10.0);
    // 推入 NaN -> 变为 0.0
    bool ret = analyzer.push(std::numeric_limits<double>::quiet_NaN());

    EXPECT_TRUE(ret) << "Zero策略下应返回 true";

    // 此时点集应为 {0.0, 10.0}，Gap = 5.0
    EXPECT_DOUBLE_EQ(analyzer.get_mean_persistence(), 5.0);

    // 再次推入 Inf -> 变为 0.0
    // 点集 {0.0, 10.0} (重复的0被set去重)，但时间窗口中有 [10, 0, 0]
    analyzer.push(std::numeric_limits<double>::infinity());

    EXPECT_DOUBLE_EQ(analyzer.get_mean_persistence(), 5.0);
}

/**
 * @brief 策略测试 3: NoCheckBadValuePolicy
 * NaN 直接进入系统，但可能导致未定义行为（因为 NaN 无法正确排序）
 * 主要验证程序不会崩溃
 */
TEST(PolicyTest, NoCheckPolicyBehavior) {
    SlidingWindowPersistenceH0<double, NoCheckBadValuePolicy> analyzer(5);

    analyzer.push(10.0);
    bool ret = analyzer.push(std::numeric_limits<double>::quiet_NaN());

    EXPECT_TRUE(ret) << "NoCheck策略下应返回 true";

    // 由于 NaN 无法正确排序，结果可能是未定义的
    // 我们只验证程序不会崩溃，而不验证具体数值
    double res = analyzer.get_mean_persistence();
    EXPECT_NO_THROW(analyzer.get_mean_persistence());

    // 验证窗口大小增加
    EXPECT_EQ(analyzer.size(), 2);
}

// ==========================================
// 边界与异常情况测试
// ==========================================

/**
 * @brief 边界测试: 重复值逻辑 (Known Constraint)
 * 验证当前实现下，std::set 去重导致的窗口状态行为
 * 重点测试时间窗口与有序集合的一致性
 */
TEST_F(SlidingWindowH0Test, DuplicateValuesBehavior) {
    analyzer->reset(3);

    // 1. [10]
    analyzer->push(10.0);
    // 2. [10, 10] -> Set: {10}. Mean = 0
    analyzer->push(10.0);
    EXPECT_EQ(analyzer->size(), 2);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.0);

    // 3. [10, 10, 20] -> Set: {10, 20}. Gap=5
    analyzer->push(20.0);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 5.0);

    // 4. [10, 20, 30] (移除了最老的10)
    // 关键点：移除 old=10 时，remove_point_from_sorted 会把 Set 里的 10 删掉。
    // 尽管窗口里还有一个 10 (第二个)，但 Set 里已经没了。
    // 此时 Set: {20, 30}. Gap: 5.
    // 实际上第二个 10 在拓扑分析中"丢失"了。
    analyzer->push(30.0);

    // 验证当前代码行为是否稳定（不崩溃）
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 5.0);
}

/**
 * @brief 测试用例 5: 空窗口和单点边界条件
 * 验证没有数据或只有一个数据点时的正确行为
 * 数学原理：至少需要2个点才能形成间隙
 */
TEST_F(SlidingWindowH0Test, EmptyAndSinglePoint) {
    // 初始状态
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.0);
    EXPECT_EQ(analyzer->size(), 0);
    EXPECT_EQ(analyzer->valid_gap_count(), 0);

    // 单点状态
    analyzer->push(5.0);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.0);
    EXPECT_EQ(analyzer->size(), 1);
    EXPECT_EQ(analyzer->valid_gap_count(), 0);

    // 清空测试
    analyzer->clear();
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.0);
    EXPECT_EQ(analyzer->size(), 0);
}

/**
 * @brief 测试用例 6: 负数和零值处理
 * 验证算法对负数和零值的正确处理能力
 * 数学原理：距离计算使用绝对值，符号不影响间隙大小
 */
TEST_F(SlidingWindowH0Test, NegativeAndZeroValues) {
    analyzer->reset(4);

    // 包含负数、零、正数的混合数据
    analyzer->push_points({-2.0, -1.0, 0.0, 1.0});

    // 排序后为 [-2, -1, 0, 1]，所有间隙都是0.5
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.5);
    EXPECT_EQ(analyzer->valid_gap_count(), 3);
}

/**
 * @brief 测试用例 7: 大数和小数精度测试
 * 验证算法在不同数量级数据下的数值稳定性
 * 测试浮点精度对计算结果的影响
 */
TEST_F(SlidingWindowH0Test, LargeAndSmallNumbers) {
    // 大数测试
    {
        SlidingWindowPersistenceH0<double> large_analyzer(3);
        large_analyzer.push_points({1e10, 2e10, 3e10});
        EXPECT_NEAR(large_analyzer.get_mean_persistence(), 5e9, 1e-5);
    }

    // 小数测试
    {
        SlidingWindowPersistenceH0<double> small_analyzer(3);
        small_analyzer.push_points({1e-10, 2e-10, 3e-10});
        EXPECT_NEAR(small_analyzer.get_mean_persistence(), 5e-11, 1e-20);
    }
}

/**
 * @brief 测试用例 8: 批量操作性能测试
 * 验证批量添加点的正确性和效率
 * 测试大数据量下的稳定性
 */
TEST_F(SlidingWindowH0Test, BulkOperations) {
    analyzer->reset(1000);

    std::vector<double> large_dataset;
    for (int i = 0; i < 2000; ++i) {
        large_dataset.push_back(static_cast<double>(i));
    }

    int added = analyzer->push_points(large_dataset);
    EXPECT_EQ(added, 2000);
    EXPECT_EQ(analyzer->size(), 1000); // 窗口大小限制
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.5);
}

/**
 * @brief 测试用例 9: 获取点数据功能
 * 验证获取时间顺序和数值顺序点列表的正确性
 * 测试数据一致性和排序正确性
 */
TEST_F(SlidingWindowH0Test, GetPointsFunctions) {
    analyzer->push_points({5.0, 1.0, 3.0, 2.0, 4.0});

    auto time_order = analyzer->get_points_time_order();
    auto sorted = analyzer->get_points_sorted();

    // 验证大小
    EXPECT_EQ(time_order.size(), 5);
    EXPECT_EQ(sorted.size(), 5);

    // 验证时间顺序
    EXPECT_DOUBLE_EQ(time_order[0], 5.0);
    EXPECT_DOUBLE_EQ(time_order[1], 1.0);
    EXPECT_DOUBLE_EQ(time_order[2], 3.0);

    // 验证数值顺序
    EXPECT_DOUBLE_EQ(sorted[0], 1.0);
    EXPECT_DOUBLE_EQ(sorted[1], 2.0);
    EXPECT_DOUBLE_EQ(sorted[2], 3.0);
    EXPECT_DOUBLE_EQ(sorted[3], 4.0);
    EXPECT_DOUBLE_EQ(sorted[4], 5.0);
}

/**
 * @brief 测试用例 10: 无限窗口测试
 * 验证窗口大小为0时的无限窗口行为
 * 测试数据累积而不移除的情况
 */
TEST_F(SlidingWindowH0Test, InfiniteWindow) {
    SlidingWindowPersistenceH0<double> infinite_analyzer(0); // 无限窗口

    for (int i = 0; i < 100; ++i) {
        infinite_analyzer.push(static_cast<double>(i));
    }

    EXPECT_EQ(infinite_analyzer.size(), 100);
    EXPECT_DOUBLE_EQ(infinite_analyzer.get_mean_persistence(), 0.5);
    EXPECT_FALSE(infinite_analyzer.ready()); // 无限窗口永远不会ready
}

/**
 * @brief 测试用例 11: 重置功能测试
 * 验证reset方法能正确重置分析器状态
 * 测试重置后的初始状态和行为
 */
TEST_F(SlidingWindowH0Test, ResetFunctionality) {
    // 先添加一些数据
    analyzer->push_points({1.0, 2.0, 3.0, 4.0, 5.0});
    EXPECT_EQ(analyzer->size(), 5);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.5);

    // 重置为不同大小
    analyzer->reset(3);
    EXPECT_EQ(analyzer->size(), 0);
    EXPECT_EQ(analyzer->window_size(), 3);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.0);

    // 重置后重新使用
    analyzer->push_points({10.0, 20.0});
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 5.0);
}

/**
 * @brief 测试用例 12: 就绪状态测试
 * 验证ready方法的正确性
 * 测试窗口满状态和无限窗口状态
 */
TEST_F(SlidingWindowH0Test, ReadyState) {
    // 有限窗口测试
    analyzer->reset(3);
    EXPECT_FALSE(analyzer->ready());

    analyzer->push(1.0);
    EXPECT_FALSE(analyzer->ready());

    analyzer->push(2.0);
    EXPECT_FALSE(analyzer->ready());

    analyzer->push(3.0);
    EXPECT_TRUE(analyzer->ready());

    // 无限窗口测试
    SlidingWindowPersistenceH0<double> infinite_analyzer(0);
    for (int i = 0; i < 100; ++i) {
        infinite_analyzer.push(static_cast<double>(i));
        EXPECT_FALSE(infinite_analyzer.ready());
    }
}

// ==========================================
// 数学正确性验证测试
// ==========================================

/**
 * @brief 数学正确性测试: 等间距序列
 * 验证等间距点的间隙计算正确性
 * 数学预期：所有间隙相等，平均值等于单个间隙值
 */
TEST_F(SlidingWindowH0Test, EquidistantSequence) {
    analyzer->reset(5);

    // 等间距点：间隔为2
    analyzer->push_points({0.0, 2.0, 4.0, 6.0, 8.0});

    // 所有间隙应为1.0
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 1.0);
    EXPECT_EQ(analyzer->valid_gap_count(), 4);
}

/**
 * @brief 数学正确性测试: 非等间距序列
 * 验证不同间距的加权平均值计算正确性
 * 手动计算验证数学公式
 */
TEST_F(SlidingWindowH0Test, NonEquidistantSequence) {
    analyzer->reset(4);

    // 非等间距点
    analyzer->push_points({1.0, 4.0, 6.0, 10.0});

    // 间隙: (4-1)/2=1.5, (6-4)/2=1.0, (10-6)/2=2.0
    // 平均值: (1.5 + 1.0 + 2.0) / 3 = 1.5
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 1.5);
}

/**
 * @brief 数学正确性测试: 单一大间隙
 * 验证大间隙被正确阈值过滤的情况
 * 测试阈值对统计结果的影响
 */
TEST_F(SlidingWindowH0Test, SingleLargeGap) {
    analyzer->reset(2);
    analyzer->update_eps_max(10.0); // 设置阈值

    // 大间隙点
    analyzer->push_points({1.0, 100.0});

    // 间隙: (100-1)/2=49.5 > 10，应该被过滤
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 0.0);

    // 放宽阈值后应该计入
    analyzer->update_eps_max(100.0);
    EXPECT_DOUBLE_EQ(analyzer->get_mean_persistence(), 49.5);
}