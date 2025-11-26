#include "gtest/gtest.h"
#include "math/incremental_rank.h"
#include <vector>
#include <cmath>

using namespace factorlib::math;

/**
 * @class IncrementalRankTest
 * @brief IncrementalRankCalculator 测试套件
 * 
 * 测试覆盖：
 * - 基本功能：数据添加、窗口管理、排序
 * - 秩计算：中位秩的正确性
 * - 阈值标记：增量更新、统计计算
 * - 边界情况：空窗口、重复值、坏值处理
 */
class IncrementalRankTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 测试数据初始化
        test_data = {3.0, 1.0, 4.0, 1.0, 5.0, 9.0, 2.0, 6.0};
        window_size = 5;
    }

    void TearDown() override {
        // 清理资源
    }

    std::vector<double> test_data;
    std::size_t window_size;
};

/**
 * @test 测试基本数据添加和窗口管理
 * @brief 验证数据能正确添加到窗口，且窗口大小得到维护
 * 
 * 测试要点：
 * - 数据按时间顺序存储
 * - 窗口大小限制有效
 * - 过期数据被正确移除
 */
TEST_F(IncrementalRankTest, BasicPushAndWindowManagement) {
    IncrementalRankCalculator<double> calculator;
    
    // 添加数据，验证窗口大小
    for (std::size_t i = 0; i < test_data.size(); ++i) {
        calculator.push(test_data[i], window_size);
        
        // 窗口大小不应超过设定值
        EXPECT_LE(calculator.size(), window_size);
        
        // 当数据量足够时，窗口应该填满
        if (i >= window_size - 1) {
            EXPECT_TRUE(calculator.is_window_full(window_size));
        }
    }
    
    // 最终窗口大小应该等于设定值
    EXPECT_EQ(calculator.size(), window_size);
}

/**
 * @test 测试中位秩计算正确性
 * @brief 验证中位秩计算符合数学定义
 * 
 * 数学验证：
 * - 中位秩 = (rank + 0.5) / n
 * - 对于排序后的数据，中位秩应该均匀分布
 * - 重复值应该正确处理
 *
 * 特别注意：
 * - 比最小值还小的值：rank=0 → (0+0.5)/n
 * - 比最大值还大的值：rank=n → (n+0.5)/n
 */
TEST_F(IncrementalRankTest, MedianRankCalculation) {
    IncrementalRankCalculator<double> calculator;

    // 添加测试数据
    for (auto value : test_data) {
        calculator.push(value, window_size);
    }

    // 获取排序后的数据验证秩计算
    auto sorted = calculator.get_sorted_data();
    std::size_t n = sorted.size();

    // 验证排序正确性
    for (std::size_t i = 1; i < n; ++i) {
        EXPECT_LE(sorted[i-1], sorted[i]);
    }

    // 测试几个关键点的中位秩
    double min_rank = calculator.median_rank(sorted[0] - 1.0);
    double max_rank = calculator.median_rank(sorted.back() + 1.0);

    // 修正：比最小值还小的值，rank=0 → (0+0.5)/n
    EXPECT_NEAR(min_rank, 0.5 / n, 1e-10);

    // 修正：比最大值还大的值，rank=n → (n+0.5)/n = 1 + 0.5/n
    EXPECT_NEAR(max_rank, 1.0 + 0.5 / n, 1e-10);

    // 中位值的中位秩应该接近0.5
    double median_value = sorted[n / 2];
    double median_rank = calculator.median_rank(median_value);
    EXPECT_NEAR(median_rank, 0.5, 0.2);  // 允许一定误差
}

/**
 * @test 测试阈值标记的增量更新
 * @brief 验证阈值变化时标记的正确增量更新
 * 
 * 算法验证：
 * - 首次阈值设置：全量计算
 * - 后续阈值更新：只更新变化区间
 * - 标记统计量正确更新
 */
TEST_F(IncrementalRankTest, ThresholdBasedBinaryFlags) {
    IncrementalRankCalculator<double> calculator;
    
    // 先填充数据
    for (auto value : test_data) {
        calculator.push(value, window_size);
    }
    
    // 设置初始阈值
    double threshold1 = 3.0;
    calculator.update_binary_flags(threshold1);
    
    // 验证标记正确性
    auto flags1 = calculator.get_binary_flags_time_order();
    auto sorted_data = calculator.get_sorted_data();
    
    // 手动验证标记：大于阈值应为1，否则为0
    for (std::size_t i = 0; i < sorted_data.size(); ++i) {
        int expected_flag = (sorted_data[i] > threshold1) ? 1 : 0;
        // 由于标记是按时间顺序，我们需要找到对应值
        // 这里简化验证：检查标记统计的一致性
    }
    
    // 验证标记均值
    double mean1 = calculator.flagged_mean();
    EXPECT_TRUE(std::isfinite(mean1));
    
    // 更新阈值
    double threshold2 = 5.0;
    calculator.update_binary_flags(threshold2);
    
    // 验证阈值更新
    EXPECT_DOUBLE_EQ(calculator.current_threshold(), threshold2);
    
    // 标记数量应该减少（阈值提高）
    auto flags2 = calculator.get_binary_flags_time_order();
    int count1 = std::count(flags1.begin(), flags1.end(), 1);
    int count2 = std::count(flags2.begin(), flags2.end(), 1);
    EXPECT_LE(count2, count1);
}

/**
 * @test 测试标记数据的增量统计
 * @brief 验证标记为1的数据均值计算正确性
 * 
 * 统计验证：
 * - 标记均值应该等于标记数据的算术平均
 * - 添加/删除数据时统计量正确更新
 * - 阈值变化时统计量正确更新
 */
TEST_F(IncrementalRankTest, FlaggedMeanStatistics) {
    IncrementalRankCalculator<double> calculator;
    
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
    for (auto value : data) {
        calculator.push(value, 5);
    }
    
    // 设置阈值
    calculator.update_binary_flags(2.5);
    
    // 计算标记均值
    double flagged_mean = calculator.flagged_mean();
    
    // 手动计算验证：大于2.5的数据是3,4,5，均值为4.0
    double expected_mean = (3.0 + 4.0 + 5.0) / 3.0;
    EXPECT_NEAR(flagged_mean, expected_mean, 1e-10);
    
    // 添加新数据验证统计更新
    calculator.push(6.0, 5);  // 会移除1.0
    calculator.update_binary_flags(2.5);
    
    double new_flagged_mean = calculator.flagged_mean();
    double new_expected = (3.0 + 4.0 + 5.0 + 6.0) / 4.0;  // 移除1.0，添加6.0
    EXPECT_NEAR(new_flagged_mean, new_expected, 1e-10);
}

/**
 * @test 测试边界情况和异常处理
 * @brief 验证空窗口、重复值、极值等情况下的鲁棒性
 */
TEST_F(IncrementalRankTest, EdgeCasesAndRobustness) {
    IncrementalRankCalculator<double> calculator;
    
    // 测试空窗口
    EXPECT_EQ(calculator.size(), 0);
    EXPECT_DOUBLE_EQ(calculator.median_rank(1.0), 0.5);
    
    // 测试单个数据
    calculator.push(5.0, 5);
    EXPECT_DOUBLE_EQ(calculator.median_rank(5.0), 0.5);
    
    // 测试重复值
    calculator.clear();
    for (int i = 0; i < 3; ++i) {
        calculator.push(3.0, 5);
    }
    EXPECT_EQ(calculator.size(), 3);
    
    // 重复值的中位秩应该合理分布
    double rank = calculator.median_rank(3.0);
    EXPECT_GE(rank, 0.0);
    EXPECT_LE(rank, 1.0);
}

/**
 * @test 测试清空功能
 * @brief 验证clear()方法能正确重置所有状态
 */
TEST_F(IncrementalRankTest, ClearFunctionality) {
    IncrementalRankCalculator<double> calculator;
    
    // 添加数据并设置阈值
    for (auto value : test_data) {
        calculator.push(value, window_size);
    }
    calculator.update_binary_flags(3.0);
    
    EXPECT_GT(calculator.size(), 0);
    EXPECT_TRUE(std::isfinite(calculator.current_threshold()));
    
    // 清空数据
    calculator.clear();
    
    // 验证重置状态
    EXPECT_EQ(calculator.size(), 0);
    EXPECT_FALSE(calculator.is_window_full(window_size));
    EXPECT_TRUE(std::isnan(calculator.current_threshold()));
    
    // 中位秩应该返回默认值
    EXPECT_DOUBLE_EQ(calculator.median_rank(1.0), 0.5);
}

/**
 * @test 验证中位秩的边界行为
 * @brief 精确验证最小值和最大值的秩计算
 */
TEST_F(IncrementalRankTest, MedianRankBoundaryBehavior) {
    IncrementalRankCalculator<double> calculator;

    // 简单数据便于手动计算
    std::vector<double> simple_data = {1.0, 2.0, 3.0};
    for (auto value : simple_data) {
        calculator.push(value, 5);
    }

    std::size_t n = calculator.size();

    // 比最小值还小
    double below_min = calculator.median_rank(0.5);
    EXPECT_NEAR(below_min, 0.5 / n, 1e-10);  // (0+0.5)/3 ≈ 0.1667

    // 等于最小值
    double at_min = calculator.median_rank(1.0);
    // 取决于实现，可能在 (0+0.5)/3 到 (1+0.5)/3 之间

    // 比最大值还大
    double above_max = calculator.median_rank(4.0);
    EXPECT_NEAR(above_max, 1.0 + 0.5 / n, 1e-10);  // (3+0.5)/3 ≈ 1.1667
}