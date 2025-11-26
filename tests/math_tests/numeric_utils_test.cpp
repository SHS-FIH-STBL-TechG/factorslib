// tests/numeric_utils_test.cpp
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "math/numeric_utils.h"

using namespace factorlib::math;

/**
 * @brief NumericUtils测试夹具类
 */
class NumericUtilsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 测试数据初始化
        normal_double_ = 42.5;
        normal_int_ = 100;
        zero_ = 0;
        negative_ = -15.3;
        nan_value_ = std::numeric_limits<double>::quiet_NaN();
        inf_value_ = std::numeric_limits<double>::infinity();
        neg_inf_value_ = -std::numeric_limits<double>::infinity();
    }

    void TearDown() override {}

    // 测试数据成员
    double normal_double_;
    int normal_int_;
    int zero_;
    double negative_;
    double nan_value_;
    double inf_value_;
    double neg_inf_value_;
};

/**
 * @brief 测试安全的对数计算 - 正常值情况
 */
TEST_F(NumericUtilsTest, SafeLog_NormalValues) {
    /// @brief 测试正数的对数计算
    EXPECT_NEAR(NumericUtils<double>::safe_log(normal_double_), std::log(normal_double_), 1e-10);

    /// @brief 测试正整数的对数计算
    EXPECT_NEAR(NumericUtils<int>::safe_log(normal_int_), std::log(normal_int_), 1e-10);

    /// @brief 测试零和负数的对数计算返回最小值
    double min_log = std::log(std::numeric_limits<double>::min());
    EXPECT_DOUBLE_EQ(NumericUtils<int>::safe_log(zero_), min_log);
    EXPECT_DOUBLE_EQ(NumericUtils<double>::safe_log(negative_), min_log);
}

/**
 * @brief 测试安全的对数计算 - 坏值处理
 */
TEST_F(NumericUtilsTest, SafeLog_BadValues) {
    /// @brief 测试SkipNaNInfPolicy策略 - 跳过NaN
    auto result1 = NumericUtils<double>::safe_log<SkipNaNInfPolicy>(nan_value_);
    EXPECT_DOUBLE_EQ(result1, std::log(std::numeric_limits<double>::min()));

    /// @brief 测试SkipNaNInfPolicy策略 - 跳过Inf
    auto result2 = NumericUtils<double>::safe_log<SkipNaNInfPolicy>(inf_value_);
    EXPECT_DOUBLE_EQ(result2, std::log(std::numeric_limits<double>::min()));

    /// @brief 测试ZeroNaNInfPolicy策略 - 将NaN替换为0后计算
    double min_log = std::log(std::numeric_limits<double>::min());
    auto result3 = NumericUtils<double>::safe_log<ZeroNaNInfPolicy>(nan_value_);
    EXPECT_DOUBLE_EQ(result3, min_log);

    /// @brief 测试NoCheckBadValuePolicy策略 - 不检查坏值，让NaN参与计算
    auto result4 = NumericUtils<double>::safe_log<NoCheckBadValuePolicy>(nan_value_);
    // NaN 参与计算，由于 NaN > 0 返回 false，所以会返回最小对数
    EXPECT_DOUBLE_EQ(result4, min_log);
}

/**
 * @brief 测试安全的除法运算 - 正常值情况
 */
TEST_F(NumericUtilsTest, SafeDivide_NormalValues) {
    /// @brief 测试正常除法运算
    EXPECT_DOUBLE_EQ(NumericUtils<int>::safe_divide(10, 2), 5.0);
    EXPECT_DOUBLE_EQ(NumericUtils<double>::safe_divide(7.5, 2.5), 3.0);

    /// @brief 测试除零情况返回0
    EXPECT_DOUBLE_EQ(NumericUtils<int>::safe_divide(10, 0), 0.0);
    EXPECT_DOUBLE_EQ(NumericUtils<double>::safe_divide(7.5, 0.0), 0.0);

    /// @brief 测试零除零情况
    EXPECT_DOUBLE_EQ(NumericUtils<int>::safe_divide(0, 0), 0.0);
}

/**
 * @brief 测试安全的除法运算 - 坏值处理
 */
TEST_F(NumericUtilsTest, SafeDivide_BadValues) {
    /// @brief 测试分子为NaN的情况
    auto result1 = NumericUtils<double>::safe_divide<SkipNaNInfPolicy>(nan_value_, normal_double_);
    EXPECT_DOUBLE_EQ(result1, 0.0);

    /// @brief 测试分母为NaN的情况
    auto result2 = NumericUtils<double>::safe_divide<SkipNaNInfPolicy>(normal_double_, nan_value_);
    EXPECT_DOUBLE_EQ(result2, 0.0);

    /// @brief 测试分子分母都为NaN的情况
    auto result3 = NumericUtils<double>::safe_divide<SkipNaNInfPolicy>(nan_value_, nan_value_);
    EXPECT_DOUBLE_EQ(result3, 0.0);

    /// @brief 测试ZeroNaNInfPolicy策略
    auto result4 = NumericUtils<double>::safe_divide<ZeroNaNInfPolicy>(nan_value_, 2.0);
    EXPECT_DOUBLE_EQ(result4, 0.0);
}

/**
 * @brief 测试数值范围限制 - 正常值情况
 */
TEST_F(NumericUtilsTest, Clamp_NormalValues) {
    /// @brief 测试数值在范围内的情况
    EXPECT_EQ(NumericUtils<int>::clamp(5, 0, 10), 5);
    EXPECT_DOUBLE_EQ(NumericUtils<double>::clamp(3.14, 0.0, 5.0), 3.14);

    /// @brief 测试数值小于最小值的情况
    EXPECT_EQ(NumericUtils<int>::clamp(-5, 0, 10), 0);

    /// @brief 测试数值大于最大值的情况
    EXPECT_EQ(NumericUtils<int>::clamp(15, 0, 10), 10);

    /// @brief 测试边界值情况
    EXPECT_EQ(NumericUtils<int>::clamp(0, 0, 10), 0);
    EXPECT_EQ(NumericUtils<int>::clamp(10, 0, 10), 10);
}

/**
 * @brief 测试数值范围限制 - 坏值处理
 */
TEST_F(NumericUtilsTest, Clamp_BadValues) {
    /// @brief 测试输入值为NaN的情况
    auto result1 = NumericUtils<double>::clamp<SkipNaNInfPolicy>(nan_value_, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result1, 0.0);

    /// @brief 测试输入值为Inf的情况
    auto result2 = NumericUtils<double>::clamp<SkipNaNInfPolicy>(inf_value_, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result2, 0.0);

    /// @brief 测试ZeroNaNInfPolicy策略 - NaN替换为0后，clamp(0, 5, 15)应该返回5
    auto result3 = NumericUtils<double>::clamp<ZeroNaNInfPolicy>(nan_value_, 5.0, 15.0);
    EXPECT_DOUBLE_EQ(result3, 5.0);
}

/**
 * @brief 测试接近零判断 - 正常值情况
 */
TEST_F(NumericUtilsTest, IsNearZero_NormalValues) {
    /// @brief 测试接近零的数值
    EXPECT_TRUE(NumericUtils<double>::is_near_zero(0.0));
    // 1e-15 实际上大于 double 的 epsilon (~2.22e-16)，所以应该返回 false
    EXPECT_FALSE(NumericUtils<double>::is_near_zero(1e-15));
    // 使用更小的值来测试
    EXPECT_TRUE(NumericUtils<double>::is_near_zero(1e-20));
    EXPECT_TRUE(NumericUtils<float>::is_near_zero(1e-7f));

    /// @brief 测试不接近零的数值
    EXPECT_FALSE(NumericUtils<double>::is_near_zero(1.0));
    EXPECT_FALSE(NumericUtils<double>::is_near_zero(-0.1));
    EXPECT_FALSE(NumericUtils<int>::is_near_zero(1));

    /// @brief 测试自定义精度阈值
    EXPECT_TRUE(NumericUtils<double>::is_near_zero(0.01, 0.1));
    EXPECT_FALSE(NumericUtils<double>::is_near_zero(0.05, 0.01));
}

/**
 * @brief 测试接近零判断 - 坏值处理
 */
TEST_F(NumericUtilsTest, IsNearZero_BadValues) {
    /// @brief 测试NaN值返回false
    auto result1 = NumericUtils<double>::is_near_zero<SkipNaNInfPolicy>(nan_value_);
    EXPECT_FALSE(result1);

    /// @brief 测试Inf值返回false
    auto result2 = NumericUtils<double>::is_near_zero<SkipNaNInfPolicy>(inf_value_);
    EXPECT_FALSE(result2);

    /// @brief 测试负Inf值返回false
    auto result3 = NumericUtils<double>::is_near_zero<SkipNaNInfPolicy>(neg_inf_value_);
    EXPECT_FALSE(result3);
}

/**
 * @brief 测试线性插值 - 正常值情况
 */
TEST_F(NumericUtilsTest, Lerp_NormalValues) {
    /// @brief 测试边界值插值
    EXPECT_DOUBLE_EQ(NumericUtils<double>::lerp(0.0, 10.0, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(NumericUtils<double>::lerp(0.0, 10.0, 1.0), 10.0);

    /// @brief 测试中间值插值
    EXPECT_DOUBLE_EQ(NumericUtils<double>::lerp(0.0, 10.0, 0.5), 5.0);
    EXPECT_DOUBLE_EQ(NumericUtils<int>::lerp(0, 100, 0.3), 30.0);

    /// @brief 测试超出范围的插值参数自动限制
    EXPECT_DOUBLE_EQ(NumericUtils<double>::lerp(0.0, 10.0, -0.5), 0.0);
    EXPECT_DOUBLE_EQ(NumericUtils<double>::lerp(0.0, 10.0, 1.5), 10.0);
}

/**
 * @brief 测试线性插值 - 坏值处理
 */
TEST_F(NumericUtilsTest, Lerp_BadValues) {
    /// @brief 测试起始值为NaN的情况
    auto result1 = NumericUtils<double>::lerp<SkipNaNInfPolicy>(nan_value_, 10.0, 0.5);
    EXPECT_DOUBLE_EQ(result1, 0.0);

    /// @brief 测试结束值为NaN的情况
    auto result2 = NumericUtils<double>::lerp<SkipNaNInfPolicy>(0.0, nan_value_, 0.5);
    EXPECT_DOUBLE_EQ(result2, 0.0);

    /// @brief 测试两个值都为NaN的情况
    auto result3 = NumericUtils<double>::lerp<SkipNaNInfPolicy>(nan_value_, nan_value_, 0.5);
    EXPECT_DOUBLE_EQ(result3, 0.0);
}

/**
 * @brief 测试对数收益率计算 - 正常值情况
 */
TEST_F(NumericUtilsTest, LogReturn_NormalValues) {
    /// @brief 测试正常的价格序列
    EXPECT_NEAR(NumericUtils<double>::log_return(110.0, 100.0), std::log(1.1), 1e-10);
    EXPECT_NEAR(NumericUtils<int>::log_return(110, 100), std::log(1.1), 1e-10);

    /// @brief 测试价格下跌的情况
    EXPECT_NEAR(NumericUtils<double>::log_return(90.0, 100.0), std::log(0.9), 1e-10);

    /// @brief 测试价格不变的情况
    EXPECT_DOUBLE_EQ(NumericUtils<double>::log_return(100.0, 100.0), 0.0);

    /// @brief 测试前一期价格为0的情况
    EXPECT_DOUBLE_EQ(NumericUtils<double>::log_return(100.0, 0.0), 0.0);

    /// @brief 测试当前价格为0的情况（负无穷大，但被处理为0）
    EXPECT_DOUBLE_EQ(NumericUtils<double>::log_return(0.0, 100.0), 0.0);
}

/**
 * @brief 测试带坏值处理的对数收益率计算
 */
TEST_F(NumericUtilsTest, LogReturnWithPolicy_BadValues) {
    /// @brief 测试当前价格为NaN的情况
    auto result1 = NumericUtils<double>::log_return_with_policy<SkipNaNInfPolicy>(nan_value_, 100.0);
    EXPECT_DOUBLE_EQ(result1, 0.0);

    /// @brief 测试前一期价格为NaN的情况
    auto result2 = NumericUtils<double>::log_return_with_policy<SkipNaNInfPolicy>(110.0, nan_value_);
    EXPECT_DOUBLE_EQ(result2, 0.0);

    /// @brief 测试两个价格都为NaN的情况
    auto result3 = NumericUtils<double>::log_return_with_policy<SkipNaNInfPolicy>(nan_value_, nan_value_);
    EXPECT_DOUBLE_EQ(result3, 0.0);

    /// @brief 测试价格为Inf的情况
    auto result4 = NumericUtils<double>::log_return_with_policy<SkipNaNInfPolicy>(inf_value_, 100.0);
    EXPECT_DOUBLE_EQ(result4, 0.0);
}

/**
 * @brief 测试简单收益率计算 - 正常值情况
 */
TEST_F(NumericUtilsTest, SimpleReturn_NormalValues) {
    /// @brief 测试正常的价格上涨情况
    EXPECT_DOUBLE_EQ(NumericUtils<double>::simple_return(110.0, 100.0), 0.1);
    EXPECT_DOUBLE_EQ(NumericUtils<int>::simple_return(110, 100), 0.1);

    /// @brief 测试价格下跌的情况
    EXPECT_DOUBLE_EQ(NumericUtils<double>::simple_return(90.0, 100.0), -0.1);

    /// @brief 测试价格不变的情况
    EXPECT_DOUBLE_EQ(NumericUtils<double>::simple_return(100.0, 100.0), 0.0);

    /// @brief 测试前一期价格为0的情况
    EXPECT_DOUBLE_EQ(NumericUtils<double>::simple_return(100.0, 0.0), 0.0);
}

/**
 * @brief 测试带坏值处理的简单收益率计算
 */
TEST_F(NumericUtilsTest, SimpleReturnWithPolicy_BadValues) {
    /// @brief 测试当前价格为NaN的情况
    auto result1 = NumericUtils<double>::simple_return_with_policy<SkipNaNInfPolicy>(nan_value_, 100.0);
    EXPECT_DOUBLE_EQ(result1, 0.0);

    /// @brief 测试前一期价格为NaN的情况
    auto result2 = NumericUtils<double>::simple_return_with_policy<SkipNaNInfPolicy>(110.0, nan_value_);
    EXPECT_DOUBLE_EQ(result2, 0.0);

    /// @brief 测试两个价格都为NaN的情况
    auto result3 = NumericUtils<double>::simple_return_with_policy<SkipNaNInfPolicy>(nan_value_, nan_value_);
    EXPECT_DOUBLE_EQ(result3, 0.0);

    /// @brief 测试ZeroNaNInfPolicy策略 - NaN替换为0后，(0-100)/100 = -1
    auto result4 = NumericUtils<double>::simple_return_with_policy<ZeroNaNInfPolicy>(nan_value_, 100.0);
    EXPECT_DOUBLE_EQ(result4, -1.0);
}

/**
 * @brief 测试不同类型的一致性
 */
TEST_F(NumericUtilsTest, TypeConsistency) {
    /// @brief 测试不同数值类型计算的一致性
    EXPECT_NEAR(NumericUtils<int>::safe_log(100), NumericUtils<double>::safe_log(100.0), 1e-10);
    EXPECT_DOUBLE_EQ(NumericUtils<int>::safe_divide(10, 3), NumericUtils<double>::safe_divide(10.0, 3.0));

    /// @brief 测试收益率计算的一致性
    EXPECT_NEAR(NumericUtils<int>::log_return(150, 100),
                NumericUtils<double>::log_return(150.0, 100.0), 1e-10);
}

/**
 * @brief 测试性能关键函数的边界情况
 */
TEST_F(NumericUtilsTest, EdgeCases) {
    /// @brief 测试极大值处理
    double max_val = std::numeric_limits<double>::max();
    EXPECT_NO_THROW(NumericUtils<double>::safe_log(max_val));
    EXPECT_NO_THROW(NumericUtils<double>::safe_divide(max_val, max_val));

    /// @brief 测试极小值处理
    double min_val = std::numeric_limits<double>::min();
    EXPECT_NO_THROW(NumericUtils<double>::safe_log(min_val));
    EXPECT_NO_THROW(NumericUtils<double>::safe_divide(min_val, min_val));

    /// @brief 测试精度边界 - epsilon本身不小于epsilon，所以应该返回false
    EXPECT_FALSE(NumericUtils<double>::is_near_zero(std::numeric_limits<double>::epsilon()));
    // 测试真正的接近零的值
    EXPECT_TRUE(NumericUtils<double>::is_near_zero(0.0));
}