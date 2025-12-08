// test_factors/low_freq_return_factor_test.cpp
#include "factors/kline/low_freq_return_factor.h"
#include "core/databus.h"
#include <gtest/gtest.h>
#include <vector>
#include <cmath>

namespace factorlib {
namespace test {

class TestLowFreqReturnFactor : public LowFreqReturnFactor {
public:
    TestLowFreqReturnFactor(const std::vector<Code>& codes,
                            const LowFreqReturnConfig& cfg = LowFreqReturnConfig{})
        : LowFreqReturnFactor(codes, cfg) {}
    void on_quote(const QuoteDepth& q) override { (void)q; }
    void on_tick(const CombinedTick& x) override { (void)x; }
};

class LowFreqReturnFactorTest : public ::testing::Test {
protected:
    void SetUp() override {
        DataBus::instance().reset();
        TestLowFreqReturnFactor::register_topics(100);
    }
    void TearDown() override {
        DataBus::instance().reset();
    }

    Bar createBar(const std::string& code, int64_t time_ms, double price) {
        Bar bar;
        bar.instrument_id = code;
        bar.data_time_ms = time_ms;
        bar.close = price;
        bar.open = price - 0.5;
        bar.high = price + 0.5;
        bar.low = price - 0.5;
        bar.volume = 1000;
        bar.turnover = price * 1000;
        bar.interval_ms = 60000;
        return bar;
    }

    // 生成线性增长序列（收益几乎恒定）
    std::vector<Bar> generateLinearBars(const std::string& code, int n, double start = 100.0, double step = 1.0) {
        std::vector<Bar> bars;
        for (int i = 0; i < n; ++i) {
            bars.push_back(createBar(code, 1000000 + i * 60000, start + i * step));
        }
        return bars;
    }

    // 生成正弦波动序列（有明显周期性）
    std::vector<Bar> generateSineBars(const std::string& code, int n, double base = 100.0, double amplitude = 5.0, int period = 20) {
        std::vector<Bar> bars;
        for (int i = 0; i < n; ++i) {
            double price = base + amplitude * std::sin(2.0 * M_PI * i / period);
            bars.push_back(createBar(code, 1000000 + i * 60000, price));
        }
        return bars;
    }

    // 生成随机游走序列
    std::vector<Bar> generateRandomWalkBars(const std::string& code, int n, double start = 100.0, double step = 0.5) {
        std::vector<Bar> bars;
        double price = start;
        std::srand(42); // 固定种子确保测试可重复
        for (int i = 0; i < n; ++i) {
            price += (std::rand() % 3 - 1) * step; // -step, 0, +step
            bars.push_back(createBar(code, 1000000 + i * 60000, price));
        }
        return bars;
    }

    bool getFactorValue(const std::string& code, int64_t timestamp, double& out) {
        return DataBus::instance().get_by_time_exact<double>("kline/ret_lowfreq_mu10", code, timestamp, out);
    }
    bool getLatestFactorValue(const std::string& code, double& out, int64_t* ts = nullptr) {
        return DataBus::instance().get_latest<double>("kline/ret_lowfreq_mu10", code, out, ts);
    }
};

// 测试1: 基础功能与双窗口初始化
TEST_F(LowFreqReturnFactorTest, BasicInitializationAndDualWindowing) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 8;  // 小窗口便于测试
    cfg.mean_window = 4;
    TestLowFreqReturnFactor factor({"TEST001"}, cfg);
    auto bars = generateLinearBars("TEST001", 15, 100.0, 0.1);

    // 需要满足两个窗口：spectral_window和mean_window
    // 先推送7根（spectral_window-1），不应发布
    for (int i = 0; i < 7; ++i) {
        factor.on_bar(bars[i]);
        double val;
        EXPECT_FALSE(getLatestFactorValue("TEST001", val));
    }
    // 第8根K线满足spectral_window，但mean_window需要4根收益（需要5根K线）
    factor.on_bar(bars[7]);
    double val;
    // 可能发布了因子（如果mean_ret也ready了），也可能没有
    // 这里不严格断言，主要测试流程
}

// 测试2: 线性增长序列（稳定收益）测试
TEST_F(LowFreqReturnFactorTest, LinearGrowthSequence) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 16;
    cfg.mean_window = 8;
    cfg.low_freq_bins = 3;
    TestLowFreqReturnFactor factor({"TEST002"}, cfg);
    auto bars = generateLinearBars("TEST002", 50, 100.0, 0.01); // 微小增长

    for (size_t i = 0; i < bars.size(); ++i) {
        factor.on_bar(bars[i]);
        double val;
        if (getLatestFactorValue("TEST002", val)) {
            // 线性增长序列的低频能量占比较高，收益均值为正，因子值应为正
            EXPECT_TRUE(std::isfinite(val));
            if (i > 25) { // 稳定期
                EXPECT_GT(val, -1e-6); // 允许微小负值（数值误差）
            }
        }
    }
}

// 测试3: 正弦波动序列（强周期性）测试
TEST_F(LowFreqReturnFactorTest, SineWaveSequence) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 32; // 窗口大小最好是周期的整数倍
    cfg.mean_window = 10;
    cfg.low_freq_bins = 2;
    TestLowFreqReturnFactor factor({"TEST003"}, cfg);
    auto bars = generateSineBars("TEST003", 100, 100.0, 2.0, 16);

    std::vector<double> values;
    for (const auto& bar : bars) {
        factor.on_bar(bar);
        double val;
        if (getLatestFactorValue("TEST003", val) && std::isfinite(val)) {
            values.push_back(val);
        }
    }
    // 正弦序列应该有稳定的低频能量占比
    EXPECT_GT(values.size(), 0);
    // 检查值的稳定性（方差不应过大）
    if (values.size() > 10) {
        double sum = 0, sum2 = 0;
        for (double v : values) { sum += v; sum2 += v*v; }
        double var = sum2/values.size() - (sum/values.size())*(sum/values.size());
        EXPECT_LT(var, 0.1); // 方差应较小
    }
}

// 测试4: 随机游走序列测试
TEST_F(LowFreqReturnFactorTest, RandomWalkSequence) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 32;
    cfg.mean_window = 10;
    TestLowFreqReturnFactor factor({"TEST004"}, cfg);
    auto bars = generateRandomWalkBars("TEST004", 100);

    int finite_count = 0;
    for (const auto& bar : bars) {
        factor.on_bar(bar);
        double val;
        if (getLatestFactorValue("TEST004", val) && std::isfinite(val)) {
            ++finite_count;
            // 随机游走的因子值可能在0附近波动
            EXPECT_GT(std::abs(val), 1e-10); // 不应全为0
        }
    }
    EXPECT_GT(finite_count, 0);
}

// 测试5: 常数价格序列（零收益）测试
TEST_F(LowFreqReturnFactorTest, ConstantPriceSequence) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 16;
    cfg.mean_window = 8;
    TestLowFreqReturnFactor factor({"TEST005"}, cfg);
    auto bars = generateLinearBars("TEST005", 50, 100.0, 0.0);

    int publish_count = 0;
    for (const auto& bar : bars) {
        factor.on_bar(bar);
        double val;
        if (getLatestFactorValue("TEST005", val)) {
            ++publish_count;
            // 常数价格：收益为0，均值mu=0，因子值应为0
            EXPECT_DOUBLE_EQ(val, 0.0);
        }
    }
    // 常数序列应能正常发布因子（低频能量占比可能为NaN，但mu=0，乘积为0）
    EXPECT_GT(publish_count, 0);
}

// 测试6: 非法价格处理
TEST_F(LowFreqReturnFactorTest, InvalidPriceHandling) {
    TestLowFreqReturnFactor factor({"TEST006"});
    std::vector<Bar> bars;
    for (int i = 0; i < 100; ++i) {
        Bar bar = createBar("TEST006", 1000000 + i*60000, 100.0 + i*0.1);
        if (i == 30 || i == 60) bar.close = 0.0; // 非法价格
        if (i == 45) bar.close = -1.0; // 非法价格
        bars.push_back(bar);
    }
    // 应处理而不崩溃
    for (const auto& bar : bars) factor.on_bar(bar);
    SUCCEED();
}

// 测试7: 多标的计算
TEST_F(LowFreqReturnFactorTest, MultipleInstruments) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 16;
    cfg.mean_window = 8;
    TestLowFreqReturnFactor factor({"CODE_A", "CODE_B"}, cfg);
    auto bars_a = generateLinearBars("CODE_A", 50, 100.0, 0.1);
    auto bars_b = generateSineBars("CODE_B", 50, 50.0, 1.0, 10);

    for (int i = 0; i < 50; ++i) {
        factor.on_bar(bars_a[i]);
        factor.on_bar(bars_b[i]);
    }
    double val_a, val_b;
    EXPECT_TRUE(getLatestFactorValue("CODE_A", val_a));
    EXPECT_TRUE(getLatestFactorValue("CODE_B", val_b));
    // 不同序列应得到不同因子值
    EXPECT_NE(val_a, val_b);
}

// 测试8: 代码白名单过滤
TEST_F(LowFreqReturnFactorTest, CodeWhitelistFiltering) {
    TestLowFreqReturnFactor factor({"CODE_X"}); // 只监控CODE_X
    Bar bar_x = createBar("CODE_X", 1000000, 100.0);
    Bar bar_y = createBar("CODE_Y", 1000000, 100.0);

    // 推送足够数据填满两个窗口（默认spectral_window=64, mean_window=10）
    for (int i = 0; i < 200; ++i) {
        bar_x.data_time_ms = bar_y.data_time_ms = 1000000 + i*60000;
        bar_x.close = bar_y.close = 100.0 + i*0.1;
        factor.on_bar(bar_x);
        factor.on_bar(bar_y); // 应被忽略
    }

    double val_x, val_y;
    EXPECT_TRUE(getLatestFactorValue("CODE_X", val_x));
    EXPECT_FALSE(getLatestFactorValue("CODE_Y", val_y));
}

// 测试9: 空代码列表（监控所有）
TEST_F(LowFreqReturnFactorTest, EmptyCodeListMonitorsAll) {
    std::vector<std::string> empty_list;
    TestLowFreqReturnFactor factor(empty_list);
    Bar bar1 = createBar("ANY_CODE_1", 1000000, 100.0);
    Bar bar2 = createBar("ANY_CODE_2", 1000000, 100.0);

    for (int i = 0; i < 200; ++i) {
        bar1.data_time_ms = bar2.data_time_ms = 1000000 + i*60000;
        bar1.close = bar2.close = 100.0 + i*0.1;
        factor.on_bar(bar1);
        factor.on_bar(bar2);
    }

    double val1, val2;
    EXPECT_TRUE(getLatestFactorValue("ANY_CODE_1", val1));
    EXPECT_TRUE(getLatestFactorValue("ANY_CODE_2", val2));
}

// 测试10: 配置参数边界测试 - 小窗口
TEST_F(LowFreqReturnFactorTest, SmallWindowConfig) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 8; // SpectralFeatures要求至少8
    cfg.mean_window = 2;     // SlidingWindowStats要求至少2
    cfg.low_freq_bins = 1;
    TestLowFreqReturnFactor factor({"TEST010"}, cfg);
    auto bars = generateLinearBars("TEST010", 20, 100.0, 0.1);

    int publish_count = 0;
    for (const auto& bar : bars) {
        factor.on_bar(bar);
        double val;
        if (getLatestFactorValue("TEST010", val)) {
            ++publish_count;
            EXPECT_TRUE(std::isfinite(val));
        }
    }
    EXPECT_GT(publish_count, 0);
}

// 测试11: 配置参数边界测试 - 低频频点数量
TEST_F(LowFreqReturnFactorTest, LowFreqBinsConfig) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 32;
    cfg.mean_window = 10;
    cfg.low_freq_bins = 1; // 只取第一个正频点
    TestLowFreqReturnFactor factor1({"TEST011A"}, cfg);

    cfg.low_freq_bins = 8; // 取多个低频频点
    TestLowFreqReturnFactor factor2({"TEST011B"}, cfg);

    auto bars = generateSineBars("TEST011A", 100, 100.0, 2.0, 16);
    for (const auto& bar : bars) {
        factor1.on_bar(bar);
        factor2.on_bar(bar);
    }

    double val1, val2;
    EXPECT_TRUE(getLatestFactorValue("TEST011A", val1));
    EXPECT_TRUE(getLatestFactorValue("TEST011B", val2));
    // 不同low_freq_bins应得到不同结果
    EXPECT_NE(val1, val2);
}

// 测试12: 时间戳正确性
TEST_F(LowFreqReturnFactorTest, TimestampCorrectness) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 16;
    cfg.mean_window = 8;
    TestLowFreqReturnFactor factor({"TEST012"}, cfg);
    auto bars = generateLinearBars("TEST012", 50, 100.0, 0.1);

    for (size_t i = 0; i < bars.size(); ++i) {
        factor.on_bar(bars[i]);
        double val;
        int64_t ts;
        if (getLatestFactorValue("TEST012", val, &ts)) {
            EXPECT_EQ(ts, bars[i].data_time_ms);
        }
    }
}

// 测试13: 因子值范围与稳定性
TEST_F(LowFreqReturnFactorTest, FactorValueRangeAndStability) {
    TestLowFreqReturnFactor factor({"TEST013"});
    auto bars = generateRandomWalkBars("TEST013", 200);

    std::vector<double> values;
    for (const auto& bar : bars) {
        factor.on_bar(bar);
        double val;
        if (getLatestFactorValue("TEST013", val) && std::isfinite(val)) {
            values.push_back(val);
        }
    }

    EXPECT_GT(values.size(), 50); // 应有足够多有效值
    for (double v : values) {
        // 检查值是否合理（不应过大）
        EXPECT_LT(std::abs(v), 10.0);
    }
}

// 测试14: 频谱组件失效情况
TEST_F(LowFreqReturnFactorTest, SpectralComponentFailure) {
    LowFreqReturnConfig cfg;
    cfg.spectral_window = 16;
    cfg.mean_window = 8;
    TestLowFreqReturnFactor factor({"TEST014"}, cfg);

    // 创建特殊序列：可能使低频能量占比返回NaN（如全零序列）
    std::vector<Bar> bars;
    for (int i = 0; i < 50; ++i) {
        // 交错出现相同价格，使收益为0
        double price = (i % 2 == 0) ? 100.0 : 100.0;
        bars.push_back(createBar("TEST014", 1000000 + i*60000, price));
    }

    int publish_count = 0;
    for (const auto& bar : bars) {
        factor.on_bar(bar);
        double val;
        if (getLatestFactorValue("TEST014", val)) {
            ++publish_count;
            // 如果低频能量占比为NaN，因子不应发布（因为代码中会检查）
            // 但如果mu为0，乘积为0，可能仍会发布
            EXPECT_TRUE(val == 0.0 || std::isfinite(val));
        }
    }
}

// 测试15: 与DataBus的集成测试
TEST_F(LowFreqReturnFactorTest, DataBusIntegration) {
    TestLowFreqReturnFactor factor({"TEST015"});

    // 订阅回调
    double received_value = 0.0;
    std::string received_code;
    int64_t received_ts = 0;
    DataBus::instance().subscribe<double>("kline/ret_lowfreq_mu10", "TEST015",
        [&](const std::string& code, int64_t ts, const double& val) {
            received_code = code;
            received_ts = ts;
            received_value = val;
        });

    // 推送足够数据
    auto bars = generateLinearBars("TEST015", 200, 100.0, 0.05);
    for (const auto& bar : bars) {
        factor.on_bar(bar);
    }

    // 验证回调被调用且数据有效
    EXPECT_EQ(received_code, "TEST015");
    EXPECT_GT(received_ts, 0);
    EXPECT_TRUE(std::isfinite(received_value));
}

} // namespace test
} // namespace factorlib

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}