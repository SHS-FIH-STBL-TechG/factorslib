// tests/stat_factors_tests/wavelet_trend_energy_factor_test.cpp

#include "../../src/stat_factors/wavelet_trend_energy_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "utils/databus.h"
#include "utils/types.h"
#include "../utils/test_config.h"   // testcfg::read_bars_from_cfg

using namespace factorlib;

using namespace factorlib;

// =====================[ 测试夹具 ]=====================

class WaveletTrendEnergyFactorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 每次测试前重置 DataBus
        DataBus::instance().reset();
        WaveletTrendEnergyFactor::register_topics(1024);
    }

    void TearDown() override {
        DataBus::instance().reset();
    }

    // 辅助函数：生成测试数据
    std::vector<double> generate_constant_series(size_t n, double value) {
        return std::vector<double>(n, value);
    }

    std::vector<double> generate_linear_series(size_t n, double start, double slope) {
        std::vector<double> series;
        for (size_t i = 0; i < n; ++i) {
            series.push_back(start + slope * i);
        }
        return series;
    }

    std::vector<double> generate_sine_series(size_t n, double amplitude, double frequency) {
        std::vector<double> series;
        for (size_t i = 0; i < n; ++i) {
            series.push_back(amplitude * std::sin(2 * M_PI * frequency * i));
        }
        return series;
    }

    std::vector<double> generate_step_series(size_t n, double low, double high, size_t step_point) {
        std::vector<double> series(n, low);
        for (size_t i = step_point; i < n; ++i) {
            series[i] = high;
        }
        return series;
    }

    // 辅助函数：处理价格序列
    void process_price_series(WaveletTrendEnergyFactor& factor,
                             const std::string& code,
                             const std::vector<double>& prices,
                             int64_t start_time = 1000) {
        for (size_t i = 0; i < prices.size(); ++i) {
            // 使用 Transaction 构造 CombinedTick
            Transaction t;
            t.instrument_id = code;
            t.data_time_ms = start_time + i * 1000;
            t.price = prices[i];
            // 使用隐式转换构造函数
            factor.on_tick(t);
        }
    }

    // 辅助函数：获取最新因子值
    double get_latest_factor_value(const std::string& code) {
        double value;
        int64_t ts;
        if (DataBus::instance().get_latest<double>(TOP_WAVE_TREND, code, value, &ts)) {
            return value;
        }
        return std::numeric_limits<double>::quiet_NaN();
    }
};

// =====================[ 基础功能测试 ]=====================

TEST_F(WaveletTrendEnergyFactorTest, ConstructorAndConfig) {
    std::vector<std::string> codes = {"test001"};
    WaveTrendConfig cfg;
    cfg.window_size = 64;
    cfg.levels_J = 5;
    cfg.trend_start_j = 3;
    cfg.wavelet = "db4";

    WaveletTrendEnergyFactor factor(codes, cfg);

    EXPECT_EQ(factor.get_name(), "WaveletTrendEnergy");
    EXPECT_EQ(factor.get_codes().size(), 1);
    EXPECT_EQ(factor.get_codes()[0], "test001");
}

TEST_F(WaveletTrendEnergyFactorTest, TopicRegistration) {
    // 测试前应该没有数据
    double value;
    EXPECT_FALSE(DataBus::instance().get_latest<double>(TOP_WAVE_TREND, "any_code", value));

    // 注册后应该可以正常使用
    WaveletTrendEnergyFactor::register_topics(100);

    // 发布测试数据
    DataBus::instance().publish<double>(TOP_WAVE_TREND, "test_code", 1000, 0.5);

    // 应该能获取到数据
    int64_t ts;
    EXPECT_TRUE(DataBus::instance().get_latest<double>(TOP_WAVE_TREND, "test_code", value, &ts));
    EXPECT_DOUBLE_EQ(value, 0.5);
    EXPECT_EQ(ts, 1000);
}

// =====================[ 数值计算验证测试 ]=====================

TEST_F(WaveletTrendEnergyFactorTest, ConstantSeriesShouldHaveHighTrendEnergy) {
    std::vector<std::string> codes = {"const_series"};
    WaveTrendConfig cfg;
    cfg.window_size = 64;
    cfg.levels_J = 4;
    cfg.trend_start_j = 2;

    WaveletTrendEnergyFactor factor(codes, cfg);

    // 生成常数序列
    auto prices = generate_constant_series(100, 100.0);
    process_price_series(factor, "const_series", prices);

    // 常数序列应该具有很高的趋势能量比（接近1.0）
    double ratio = get_latest_factor_value("const_series");
    EXPECT_TRUE(std::isfinite(ratio));
    EXPECT_GE(ratio, 0.8);  // 趋势能量应该很高
    EXPECT_LE(ratio, 1.0);

    LOG_INFO("Constant series trend energy ratio: {}", ratio);
}

TEST_F(WaveletTrendEnergyFactorTest, LinearTrendSeriesShouldHaveHighTrendEnergy) {
    std::vector<std::string> codes = {"linear_series"};
    WaveTrendConfig cfg;
    cfg.window_size = 64;
    cfg.levels_J = 4;
    cfg.trend_start_j = 2;

    WaveletTrendEnergyFactor factor(codes, cfg);

    // 生成线性趋势序列
    auto prices = generate_linear_series(100, 100.0, 0.1); // 稳定上升
    process_price_series(factor, "linear_series", prices);

    // 线性趋势序列也应该具有较高的趋势能量比
    double ratio = get_latest_factor_value("linear_series");
    EXPECT_TRUE(std::isfinite(ratio));
    EXPECT_GE(ratio, 0.6);  // 线性趋势应该有较高趋势能量
    EXPECT_LE(ratio, 1.0);

    LOG_INFO("Linear series trend energy ratio: {}", ratio);
}

TEST_F(WaveletTrendEnergyFactorTest, HighFrequencyNoiseShouldHaveLowTrendEnergy) {
    std::vector<std::string> codes = {"noise_series"};
    WaveTrendConfig cfg;
    cfg.window_size = 128;  // 较大窗口以更好捕捉频率特性
    cfg.levels_J = 6;
    cfg.trend_start_j = 4;

    WaveletTrendEnergyFactor factor(codes, cfg);

    // 生成高频噪声序列
    auto prices = generate_sine_series(200, 10.0, 0.2); // 高频正弦波
    process_price_series(factor, "noise_series", prices);

    // 高频噪声序列应该具有较低的趋势能量比
    double ratio = get_latest_factor_value("noise_series");
    EXPECT_TRUE(std::isfinite(ratio));
    EXPECT_LE(ratio, 0.5);  // 高频噪声的趋势能量应该较低

    LOG_INFO("High frequency noise trend energy ratio: {}", ratio);
}

TEST_F(WaveletTrendEnergyFactorTest, StepChangeSeries) {
    std::vector<std::string> codes = {"step_series"};
    WaveTrendConfig cfg;
    cfg.window_size = 64;
    cfg.levels_J = 4;
    cfg.trend_start_j = 2;

    WaveletTrendEnergyFactor factor(codes, cfg);

    // 生成阶跃变化序列
    auto prices = generate_step_series(100, 100.0, 110.0, 50);
    process_price_series(factor, "step_series", prices);

    // 阶跃变化包含趋势和细节成分
    double ratio = get_latest_factor_value("step_series");
    EXPECT_TRUE(std::isfinite(ratio));
    // 阶跃变化的趋势能量应该在中等范围
    EXPECT_GE(ratio, 0.3);
    EXPECT_LE(ratio, 0.9);

    LOG_INFO("Step series trend energy ratio: {}", ratio);
}

// =====================[ 边界条件测试 ]=====================

TEST_F(WaveletTrendEnergyFactorTest, SmallWindowSize) {
    std::vector<std::string> codes = {"small_window"};
    WaveTrendConfig cfg;
    cfg.window_size = 16;  // 较小窗口
    cfg.levels_J = 3;      // 相应减少层数
    cfg.trend_start_j = 2;

    WaveletTrendEnergyFactor factor(codes, cfg);

    auto prices = generate_constant_series(20, 50.0);
    process_price_series(factor, "small_window", prices);

    double ratio = get_latest_factor_value("small_window");
    EXPECT_TRUE(std::isfinite(ratio));

    LOG_INFO("Small window trend energy ratio: {}", ratio);
}

TEST_F(WaveletTrendEnergyFactorTest, DifferentTrendStartLevels) {
    std::vector<std::string> codes = {"test_levels"};

    // 测试不同的趋势起始层级
    for (int trend_start = 1; trend_start <= 4; ++trend_start) {
        WaveTrendConfig cfg;
        cfg.window_size = 64;
        cfg.levels_J = 4;
        cfg.trend_start_j = trend_start;

        WaveletTrendEnergyFactor factor(codes, cfg);

        auto prices = generate_linear_series(100, 100.0, 0.05);
        process_price_series(factor, "test_levels", prices);

        double ratio = get_latest_factor_value("test_levels");
        EXPECT_TRUE(std::isfinite(ratio));

        LOG_INFO("Trend start level {} -> ratio: {}", trend_start, ratio);

        // trend_start_j 越大，趋势能量比应该越小（因为趋势定义更严格）
        // 但我们主要验证不会崩溃且输出合理值
        EXPECT_GE(ratio, 0.0);
        EXPECT_LE(ratio, 1.0);
    }
}

// =====================[ 数据完整性测试 ]=====================
TEST_F(WaveletTrendEnergyFactorTest, InvalidPriceHandling) {
    std::vector<std::string> codes = {"invalid_test"};
    WaveTrendConfig cfg;
    cfg.window_size = 32;
    cfg.levels_J = 3;

    WaveletTrendEnergyFactor factor(codes, cfg);

    // 混合有效和无效价格
    std::vector<double> prices = {100.0, -1.0, 0.0, 101.0, 102.0, 103.0};

    for (size_t i = 0; i < prices.size(); ++i) {
        Transaction t;
        t.instrument_id = "invalid_test";
        t.data_time_ms = 1000 + i * 1000;
        t.price = prices[i];
        factor.on_tick(t);
    }

    // 即使有无效价格，因子也不应该崩溃
    // 最后应该能输出有效值（因为窗口中有足够多有效价格）
    auto prices_valid = generate_constant_series(40, 105.0);
    process_price_series(factor, "invalid_test", prices_valid, 2000);

    double ratio = get_latest_factor_value("invalid_test");
    EXPECT_TRUE(std::isfinite(ratio));
}

TEST_F(WaveletTrendEnergyFactorTest, MultipleCodesIndependent) {
    std::vector<std::string> codes = {"code1", "code2", "code3"};
    WaveTrendConfig cfg;
    cfg.window_size = 32;
    cfg.levels_J = 3;

    WaveletTrendEnergyFactor factor(codes, cfg);

    // 为不同代码生成不同模式的价格序列
    auto prices1 = generate_constant_series(40, 100.0);
    auto prices2 = generate_linear_series(40, 50.0, 0.5);
    auto prices3 = generate_sine_series(40, 10.0, 0.1);

    process_price_series(factor, "code1", prices1);
    process_price_series(factor, "code2", prices2);
    process_price_series(factor, "code3", prices3);

    // 每个代码应该有独立的因子值
    double ratio1 = get_latest_factor_value("code1");
    double ratio2 = get_latest_factor_value("code2");
    double ratio3 = get_latest_factor_value("code3");

    EXPECT_TRUE(std::isfinite(ratio1));
    EXPECT_TRUE(std::isfinite(ratio2));
    EXPECT_TRUE(std::isfinite(ratio3));

    // 不同价格模式应该产生不同的趋势能量比
    LOG_INFO("Multiple codes - constant: {}, linear: {}, sine: {}",
             ratio1, ratio2, ratio3);
}

// =====================[ 不同数据源测试 ]=====================
TEST_F(WaveletTrendEnergyFactorTest, DifferentDataSources) {
    std::vector<std::string> codes = {"multi_source"};
    WaveTrendConfig cfg;
    cfg.window_size = 32;
    cfg.levels_J = 3;

    WaveletTrendEnergyFactor factor(codes, cfg);

    // 测试 QuoteDepth（中间价）
    QuoteDepth quote;
    quote.instrument_id = "multi_source";
    quote.data_time_ms = 1000;
    quote.bid_price = 99.5;
    quote.ask_price = 100.5;
    factor.on_quote(quote);

    // 测试 Bar（收盘价）
    Bar bar;
    bar.instrument_id = "multi_source";
    bar.data_time_ms = 2000;
    bar.close = 101.0;
    factor.on_bar(bar);

    // 测试 CombinedTick（通过 Transaction）
    Transaction t;
    t.instrument_id = "multi_source";
    t.data_time_ms = 3000;
    t.price = 102.0;
    factor.on_tick(t);

    // 填充足够数据使窗口就绪
    auto prices = generate_constant_series(35, 103.0); // 32 + 3次调用 = 35
    process_price_series(factor, "multi_source", prices, 4000);

    double ratio = get_latest_factor_value("multi_source");
    EXPECT_TRUE(std::isfinite(ratio));
}

// =====================[ 时间序列特性测试 ]=====================

TEST_F(WaveletTrendEnergyFactorTest, EnergyRatioRangeValidation) {
    std::vector<std::string> codes = {"range_test"};
    WaveTrendConfig cfg;
    cfg.window_size = 64;
    cfg.levels_J = 4;
    cfg.trend_start_j = 2;

    WaveletTrendEnergyFactor factor(codes, cfg);

    // 测试多种价格模式，验证输出范围
    std::vector<std::pair<std::string, std::vector<double>>> test_cases = {
        {"pure_trend", generate_linear_series(100, 100.0, 0.1)},
        {"pure_noise", generate_sine_series(100, 5.0, 0.3)},
        {"mixed", generate_linear_series(100, 100.0, 0.05)}
    };

    for (const auto& test_case : test_cases) {
        const auto& pattern = test_case.first;
        const auto& prices = test_case.second;

        process_price_series(factor, "range_test", prices);

        double ratio = get_latest_factor_value("range_test");
        EXPECT_TRUE(std::isfinite(ratio)) << "Pattern: " << pattern;
        EXPECT_GE(ratio, 0.0) << "Pattern: " << pattern;
        EXPECT_LE(ratio, 1.0) << "Pattern: " << pattern;

        LOG_INFO("Pattern '{}' energy ratio: {}", pattern, ratio);
    }
}


