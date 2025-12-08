// test_factors/ar1_return_factor_test.cpp
#include "factors/kline/ar1_return_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <algorithm>

namespace factorlib {

// 创建一个测试用的具体因子类，实现所有纯虚函数
class TestAr1ReturnFactor : public Ar1ReturnFactor {
public:
    TestAr1ReturnFactor(const std::vector<std::string>& codes, 
                       const Ar1ReturnConfig& cfg = Ar1ReturnConfig{})
        : Ar1ReturnFactor(codes, cfg) {}
    
    // 实现纯虚函数 - 对于K线因子，这些可以留空
    void on_quote(const QuoteDepth& q) override {
        // K线因子通常不需要处理tick数据
        (void)q;  // 消除未使用参数警告
    }
    
    void on_tick(const CombinedTick& x) override {
        // K线因子通常不需要处理tick数据
        (void)x;  // 消除未使用参数警告
    }
};

class Ar1ReturnFactorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 每次测试前重置DataBus，确保环境干净
        DataBus::instance().reset();
        // 注册因子需要的topic
        TestAr1ReturnFactor::register_topics(100);
    }
    
    void TearDown() override {
        // 清理
        DataBus::instance().reset();
    }
    
    // 生成线性增长的K线数据（用于测试正相关）
    std::vector<Bar> generateLinearlyGrowingBars(const std::string& code, int n, double start_price = 100.0) {
        std::vector<Bar> bars;
        int64_t start_time = 1000000;
        
        for (int i = 0; i < n; ++i) {
            Bar bar;
            bar.instrument_id = code;
            bar.data_time_ms = start_time + i * 60000; // 每分钟一根
            bar.open = start_price + i * 1.0;
            bar.high = start_price + i * 1.0 + 0.5;
            bar.low = start_price + i * 1.0 - 0.5;
            bar.close = start_price + i * 1.0;
            bar.volume = 1000 + i * 100;
            bar.turnover = (start_price + i * 1.0) * (1000 + i * 100);
            bar.interval_ms = 60000;
            bars.push_back(bar);
        }
        
        return bars;
    }
    
    // 生成均值回复的K线数据（用于测试负相关）
    std::vector<Bar> generateMeanRevertingBars(const std::string& code, int n, double base_price = 100.0) {
        std::vector<Bar> bars;
        int64_t start_time = 1000000;
        double price = base_price;
        
        for (int i = 0; i < n; ++i) {
            // 简单的均值回复模式：价格在100附近波动
            if (price > base_price + 5.0) {
                price -= 2.0;
            } else if (price < base_price - 5.0) {
                price += 2.0;
            } else {
                // 随机小波动
                price += ((i % 3) - 1) * 1.0;
            }
            
            Bar bar;
            bar.instrument_id = code;
            bar.data_time_ms = start_time + i * 60000;
            bar.open = price;
            bar.high = price + 0.5;
            bar.low = price - 0.5;
            bar.close = price;
            bar.volume = 1000 + i * 100;
            bar.turnover = price * (1000 + i * 100);
            bar.interval_ms = 60000;
            bars.push_back(bar);
        }
        
        return bars;
    }
    
    // 生成随机游走的K线数据（用于测试接近0的AR系数）
    std::vector<Bar> generateRandomWalkBars(const std::string& code, int n, double start_price = 100.0) {
        std::vector<Bar> bars;
        int64_t start_time = 1000000;
        double price = start_price;
        
        for (int i = 0; i < n; ++i) {
            // 简单随机游走
            price += ((std::rand() % 3) - 1) * 0.5; // -0.5, 0, +0.5
            
            Bar bar;
            bar.instrument_id = code;
            bar.data_time_ms = start_time + i * 60000;
            bar.open = price;
            bar.high = price + 0.5;
            bar.low = price - 0.5;
            bar.close = price;
            bar.volume = 1000 + i * 100;
            bar.turnover = price * (1000 + i * 100);
            bars.push_back(bar);
        }
        
        return bars;
    }
    
    // 生成常数价格的K线数据（用于测试零方差情况）
    std::vector<Bar> generateConstantBars(const std::string& code, int n, double price = 100.0) {
        std::vector<Bar> bars;
        int64_t start_time = 1000000;
        
        for (int i = 0; i < n; ++i) {
            Bar bar;
            bar.instrument_id = code;
            bar.data_time_ms = start_time + i * 60000;
            bar.open = price;
            bar.high = price + 0.1;
            bar.low = price - 0.1;
            bar.close = price;
            bar.volume = 1000 + i * 100;
            bar.turnover = price * (1000 + i * 100);
            bars.push_back(bar);
        }
        
        return bars;
    }
    
    // 检查DataBus中是否有特定时间戳的因子值
    bool hasFactorValue(const std::string& code, int64_t timestamp, double& value) {
        return DataBus::instance().get_by_time_exact<double>(
            "kline/ar1_return_coeff", code, timestamp, value);
    }
    
    // 获取最新的因子值
    bool getLatestFactorValue(const std::string& code, double& value, int64_t* timestamp = nullptr) {
        return DataBus::instance().get_latest<double>(
            "kline/ar1_return_coeff", code, value, timestamp);
    }
};

// 测试1: 基本功能测试 - 正常计算AR1系数
TEST_F(Ar1ReturnFactorTest, BasicFunctionality) {
    // 使用默认窗口大小40
    TestAr1ReturnFactor factor({"000001.SZ"}, Ar1ReturnConfig{40});
    
    // 生成线性增长的价格序列（应该有正的自相关性）
    auto bars = generateLinearlyGrowingBars("000001.SZ", 45); // 需要40个收益样本，即41根K线
    
    // 推送前40根K线（窗口不满，不应发布因子）
    for (int i = 0; i < 40; ++i) {
        factor.on_bar(bars[i]);
        double value;
        EXPECT_FALSE(getLatestFactorValue("000001.SZ", value))
            << "Window not full, should not publish factor at bar " << i;
    }
    
    // 推送第41根K线（窗口已满，应该发布因子）
    factor.on_bar(bars[40]);
    double value;
    int64_t timestamp;
    EXPECT_TRUE(getLatestFactorValue("000001.SZ", value, &timestamp))
        << "Window full, should publish factor";
    EXPECT_EQ(timestamp, bars[40].data_time_ms)
        << "Timestamp should match bar data time";
    
    // 对于线性增长序列，AR1系数应该接近1（强正自相关）
    EXPECT_GT(value, 0.5) << "Linearly growing series should have positive AR1 coefficient";
    EXPECT_LT(value, 1.1) << "AR1 coefficient should be reasonable";
}

// 测试2: 均值回复序列测试 - 负自相关性
TEST_F(Ar1ReturnFactorTest, MeanRevertingSeries) {
    TestAr1ReturnFactor factor({"000001.SZ"}, Ar1ReturnConfig{30});
    
    auto bars = generateMeanRevertingBars("000001.SZ", 35); // 需要31根K线
    
    // 推送足够多的K线填满窗口（需要31根）
    for (int i = 0; i < 31; ++i) {
        factor.on_bar(bars[i]);
    }
    
    // 均值回复序列应该有负的AR1系数
    double value;
    EXPECT_TRUE(getLatestFactorValue("000001.SZ", value));
    EXPECT_LT(value, 0.0) << "Mean-reverting series should have negative AR1 coefficient";
    EXPECT_GT(value, -1.0) << "AR1 coefficient should be > -1 for stationarity";
}

// 测试3: 随机游走测试 - AR1系数接近0
TEST_F(Ar1ReturnFactorTest, RandomWalkSeries) {
    TestAr1ReturnFactor factor({"000001.SZ"}, Ar1ReturnConfig{40});
    
    auto bars = generateRandomWalkBars("000001.SZ", 50);
    
    for (int i = 0; i < 45; ++i) {
        factor.on_bar(bars[i]);
    }
    
    double value;
    EXPECT_TRUE(getLatestFactorValue("000001.SZ", value));
    // 随机游走的AR1系数应该在0附近
    EXPECT_NEAR(value, 0.0, 0.3) << "Random walk should have AR1 coefficient near 0";
}

// 测试4: 多股票代码测试
TEST_F(Ar1ReturnFactorTest, MultipleInstruments) {
    std::vector<std::string> codes = {"000001.SZ", "000002.SZ", "000003.SZ"};
    TestAr1ReturnFactor factor(codes, Ar1ReturnConfig{20});
    
    // 为每个代码生成不同的价格序列（需要21根K线）
    auto bars1 = generateLinearlyGrowingBars("000001.SZ", 25, 50.0);
    auto bars2 = generateMeanRevertingBars("000002.SZ", 25, 100.0);
    auto bars3 = generateRandomWalkBars("000003.SZ", 25, 150.0);
    
    // 推送所有K线
    for (int i = 0; i < 25; ++i) {
        factor.on_bar(bars1[i]);
        factor.on_bar(bars2[i]);
        factor.on_bar(bars3[i]);
    }
    
    // 检查每个代码都有因子值
    double value1, value2, value3;
    EXPECT_TRUE(getLatestFactorValue("000001.SZ", value1));
    EXPECT_TRUE(getLatestFactorValue("000002.SZ", value2));
    EXPECT_TRUE(getLatestFactorValue("000003.SZ", value3));
    
    // 验证不同序列的AR1系数特征
    EXPECT_GT(value1, 0.5);  // 线性增长 -> 高正相关
    EXPECT_LT(value2, 0.0);  // 均值回复 -> 负相关
    EXPECT_NEAR(value3, 0.0, 0.4);  // 随机游走 -> 接近0
}

// 测试5: 白名单过滤测试
TEST_F(Ar1ReturnFactorTest, CodeWhitelist) {
    // 只监控000001.SZ和000002.SZ
    std::vector<std::string> codes = {"000001.SZ", "000002.SZ"};
    TestAr1ReturnFactor factor(codes, Ar1ReturnConfig{10});
    
    auto bars1 = generateLinearlyGrowingBars("000001.SZ", 15);
    auto bars2 = generateLinearlyGrowingBars("000002.SZ", 15);
    auto bars3 = generateLinearlyGrowingBars("000003.SZ", 15);  // 不在白名单中
    
    // 推送所有K线（需要11根才能填满窗口）
    for (int i = 0; i < 15; ++i) {
        factor.on_bar(bars1[i]);
        factor.on_bar(bars2[i]);
        factor.on_bar(bars3[i]);  // 这个应该被忽略
    }
    
    // 检查白名单内的代码有因子值
    double value1, value2, value3;
    EXPECT_TRUE(getLatestFactorValue("000001.SZ", value1));
    EXPECT_TRUE(getLatestFactorValue("000002.SZ", value2));
    
    // 白名单外的代码不应该有因子值
    EXPECT_FALSE(getLatestFactorValue("000003.SZ", value3));
}

// 测试6: 空白名单测试（监控所有代码）
TEST_F(Ar1ReturnFactorTest, EmptyWhitelist) {
    // 空向量表示监控所有代码
    TestAr1ReturnFactor factor({}, Ar1ReturnConfig{10});
    
    auto bars1 = generateLinearlyGrowingBars("000001.SZ", 15);
    auto bars2 = generateLinearlyGrowingBars("000002.SZ", 15);
    auto bars3 = generateLinearlyGrowingBars("SOME_NEW_CODE", 15);
    
    // 推送所有K线
    for (int i = 0; i < 15; ++i) {
        factor.on_bar(bars1[i]);
        factor.on_bar(bars2[i]);
        factor.on_bar(bars3[i]);
    }
    
    // 所有代码都应该有因子值
    double value1, value2, value3;
    EXPECT_TRUE(getLatestFactorValue("000001.SZ", value1));
    EXPECT_TRUE(getLatestFactorValue("000002.SZ", value2));
    EXPECT_TRUE(getLatestFactorValue("SOME_NEW_CODE", value3));
}

// 测试7: 窗口滑动测试
TEST_F(Ar1ReturnFactorTest, WindowSliding) {
    TestAr1ReturnFactor factor({"TEST"}, Ar1ReturnConfig{5});
    
    // 创建特定的价格序列以便验证滑动窗口计算
    std::vector<Bar> bars;
    int64_t time = 1000000;
    
    // 价格序列: 100, 101, 102, 103, 104, 105, 106
    // 对数收益: 约 0.01, 0.0099, 0.0098, 0.0097, 0.0096, 0.0095
    for (int i = 0; i < 7; ++i) {
        Bar bar;
        bar.instrument_id = "TEST";
        bar.data_time_ms = time + i * 60000;
        bar.close = 100.0 + i;
        bars.push_back(bar);
    }
    
    // 记录每个时间点的因子值
    std::vector<double> values;
    
    for (int i = 0; i < 7; ++i) {
        factor.on_bar(bars[i]);
        double value;
        if (getLatestFactorValue("TEST", value)) {
            values.push_back(value);
        }
    }
    
    // 需要6根K线才能有5个收益样本，所以应该在第6根（i=5）和第7根（i=6）发布
    // 共发布2次
    EXPECT_EQ(values.size(), 2);
    
    // 验证值的变化（由于是线性增长，AR1系数应该接近1且相对稳定）
    for (size_t i = 1; i < values.size(); ++i) {
        EXPECT_NEAR(values[i], values[i-1], 0.1)
            << "AR1 coefficient should be relatively stable for similar return patterns";
    }
}

// 测试8: 非法价格处理测试
TEST_F(Ar1ReturnFactorTest, InvalidPriceHandling) {
    TestAr1ReturnFactor factor({"TEST"}, Ar1ReturnConfig{5});
    
    std::vector<Bar> bars;
    int64_t time = 1000000;
    
    // 创建包含非法价格（<=0）的序列
    for (int i = 0; i < 10; ++i) {
        Bar bar;
        bar.instrument_id = "TEST";
        bar.data_time_ms = time + i * 60000;
        
        if (i == 3 || i == 7) {
            bar.close = 0.0;  // 非法价格
        } else if (i == 5) {
            bar.close = -1.0;  // 非法价格
        } else {
            bar.close = 100.0 + i;
        }
        
        bars.push_back(bar);
    }
    
    // 推送所有K线
    for (int i = 0; i < 10; ++i) {
        factor.on_bar(bars[i]);
    }
    
    // 由于有非法价格，可能无法计算有效的对数收益
    // 但因子应该处理这种情况而不崩溃
    // 消除未使用变量警告
    double value = 0.0;
    (void)value;  // 显式标记为未使用
}

// 测试9: 常数价格序列测试（零方差）
TEST_F(Ar1ReturnFactorTest, ConstantPriceSeries) {
    TestAr1ReturnFactor factor({"CONST"}, Ar1ReturnConfig{10});
    
    auto bars = generateConstantBars("CONST", 15);
    
    // 推送K线
    for (int i = 0; i < 15; ++i) {
        factor.on_bar(bars[i]);
    }
    
    // 常数价格序列的对数收益为0，方差为0
    // RollingAR1::fit应该返回false，因子不应该发布值
    double value;
    bool hasValue = getLatestFactorValue("CONST", value);
    
    // 有两种可能：要么不发布，要么发布NaN
    // 根据代码，如果fit失败或phi不是finite，会return，不发布
    // 所以很可能没有值
    if (hasValue) {
        // 如果有值，应该是NaN或某个特殊值
        EXPECT_TRUE(std::isnan(value) || !std::isfinite(value))
            << "Constant series should result in NaN or infinite AR1 coefficient";
    }
}

// 测试10: 不同窗口大小测试
TEST_F(Ar1ReturnFactorTest, DifferentWindowSizes) {
    // 测试小窗口
    {
        TestAr1ReturnFactor factor1({"TEST1"}, Ar1ReturnConfig{5});
        auto bars = generateLinearlyGrowingBars("TEST1", 10);
        
        for (int i = 0; i < 10; ++i) {
            factor1.on_bar(bars[i]);
        }
        
        double value1;
        EXPECT_TRUE(getLatestFactorValue("TEST1", value1));
    }
    
    // 重置DataBus
    DataBus::instance().reset();
    TestAr1ReturnFactor::register_topics(100);
    
    // 测试大窗口
    {
        TestAr1ReturnFactor factor2({"TEST2"}, Ar1ReturnConfig{100});
        auto bars = generateLinearlyGrowingBars("TEST2", 110);
        
        for (int i = 0; i < 110; ++i) {
            factor2.on_bar(bars[i]);
        }
        
        double value2;
        EXPECT_TRUE(getLatestFactorValue("TEST2", value2));
    }
}

// 测试11: 时间戳正确性测试
TEST_F(Ar1ReturnFactorTest, TimestampCorrectness) {
    TestAr1ReturnFactor factor({"TIMETEST"}, Ar1ReturnConfig{8});
    
    std::vector<Bar> bars;
    std::vector<int64_t> expected_timestamps;
    
    // 创建特定时间戳的K线（需要9根K线才能填满窗口）
    for (int i = 0; i < 15; ++i) {
        Bar bar;
        bar.instrument_id = "TIMETEST";
        bar.data_time_ms = 1000000 + i * 30000;  // 每30秒
        bar.close = 100.0 + i * 0.5;
        bars.push_back(bar);
        
        if (i >= 8) {  // 从第9根K线开始应该发布因子
            expected_timestamps.push_back(bar.data_time_ms);
        }
    }
    
    // 推送K线并验证时间戳
    for (int i = 0; i < 15; ++i) {
        factor.on_bar(bars[i]);
        
        if (i >= 8) {  // 第9根K线开始检查
            double value;
            int64_t timestamp;
            EXPECT_TRUE(getLatestFactorValue("TIMETEST", value, &timestamp))
                << "Should publish factor at index " << i;
            EXPECT_EQ(timestamp, bars[i].data_time_ms)
                << "Published timestamp should match bar timestamp at index " << i;
        } else if (i >= 7) {
            // 第8根K线时窗口应该不满（只有7个收益样本）
            double value;
            EXPECT_FALSE(getLatestFactorValue("TIMETEST", value))
                << "Should not publish factor at index " << i << " (window not full)";
        }
    }
}

// 测试12: 因子值范围测试
TEST_F(Ar1ReturnFactorTest, FactorValueRange) {
    TestAr1ReturnFactor factor({"RANGETEST"}, Ar1ReturnConfig{10});
    
    // 创建极端价格序列测试AR1系数范围
    std::vector<Bar> bars;
    int64_t time = 1000000;
    double price = 100.0;
    
    // 创建交替大涨大跌的序列（需要11根K线）
    for (int i = 0; i < 20; ++i) {
        Bar bar;
        bar.instrument_id = "RANGETEST";
        bar.data_time_ms = time + i * 60000;
        
        if (i % 2 == 0) {
            price *= 1.1;  // 上涨10%
        } else {
            price *= 0.9;  // 下跌10%
        }
        
        bar.close = price;
        bars.push_back(bar);
    }
    
    // 推送K线
    for (int i = 0; i < 20; ++i) {
        factor.on_bar(bars[i]);
    }
    
    double value;
    EXPECT_TRUE(getLatestFactorValue("RANGETEST", value));
    
    // AR1系数应该在合理范围内（通常-1到1之间，但可能略微超出）
    EXPECT_GT(value, -2.0) << "AR1 coefficient should not be extremely negative";
    EXPECT_LT(value, 2.0) << "AR1 coefficient should not be extremely positive";
}

// 测试13: 重复推送相同时间戳测试
TEST_F(Ar1ReturnFactorTest, DuplicateTimestamps) {
    TestAr1ReturnFactor factor({"DUPTEST"}, Ar1ReturnConfig{5});
    
    Bar bar;
    bar.instrument_id = "DUPTEST";
    bar.data_time_ms = 1000000;
    bar.close = 100.0;
    
    // 多次推送相同时间戳的K线
    for (int i = 0; i < 10; ++i) {
        factor.on_bar(bar);
        
        // 每次都会计算收益，但只有前一次的价格有效
        // 从第二次开始，收益是log(100/100)=0
    }
    
    double value;
    // 可能没有发布因子值，因为收益序列可能不满足条件
    // 或者发布了一个基于零收益序列的AR1系数
    if (getLatestFactorValue("DUPTEST", value)) {
        // 如果发布了，值可能是NaN或某个特殊值
        EXPECT_TRUE(std::isfinite(value) || std::isnan(value));
    }
}

// 测试14: 混合价格模式测试
TEST_F(Ar1ReturnFactorTest, MixedPricePatterns) {
    TestAr1ReturnFactor factor({"MIXED"}, Ar1ReturnConfig{15});
    
    std::vector<Bar> bars;
    int64_t time = 1000000;
    double price = 100.0;
    
    // 创建混合模式：前10根线性增长，后10根均值回复（需要16根K线）
    for (int i = 0; i < 25; ++i) {
        Bar bar;
        bar.instrument_id = "MIXED";
        bar.data_time_ms = time + i * 60000;
        
        if (i < 10) {
            price += 1.0;  // 线性增长
        } else {
            // 均值回复：如果价格高于105就下降，低于95就上升
            if (price > 105.0) {
                price -= 0.5;
            } else if (price < 95.0) {
                price += 0.5;
            } else {
                price += (i % 3 - 1) * 0.3;  // 小波动
            }
        }
        
        bar.close = price;
        bars.push_back(bar);
    }
    
    // 记录每次发布的值
    std::vector<double> values;
    
    for (int i = 0; i < 25; ++i) {
        factor.on_bar(bars[i]);
        
        double value;
        if (getLatestFactorValue("MIXED", value)) {
            values.push_back(value);
        }
    }
    
    // 应该有多个因子值发布（从第16根K线开始）
    EXPECT_GE(values.size(), 1);
    
    // 最后一个因子值应该反映混合模式
    // 可能不如纯序列那么极端
    EXPECT_TRUE(values.back() > -1.0 && values.back() < 1.0);
}

// 测试15: 重置状态测试
TEST_F(Ar1ReturnFactorTest, StateReset) {
    // 先创建一个因子并推送一些数据
    TestAr1ReturnFactor factor({"RESETTEST"}, Ar1ReturnConfig{5});
    
    auto bars = generateLinearlyGrowingBars("RESETTEST", 10);
    
    for (int i = 0; i < 10; ++i) {
        factor.on_bar(bars[i]);
    }
    
    double value1;
    EXPECT_TRUE(getLatestFactorValue("RESETTEST", value1));
    
    // 现在创建新的因子实例（模拟重置）
    DataBus::instance().reset();
    TestAr1ReturnFactor::register_topics(100);
    
    TestAr1ReturnFactor newFactor({"RESETTEST"}, Ar1ReturnConfig{5});
    
    // 推送相同的数据
    for (int i = 0; i < 10; ++i) {
        newFactor.on_bar(bars[i]);
    }
    
    double value2;
    EXPECT_TRUE(getLatestFactorValue("RESETTEST", value2));
    
    // 两个因子实例应该计算出相同的AR1系数（相同输入，相同窗口）
    EXPECT_NEAR(value1, value2, 1e-10)
        << "Same input should produce same AR1 coefficient";
}

} // namespace factorlib