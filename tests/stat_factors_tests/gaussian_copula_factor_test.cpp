// tests/gaussian_copula_factor_test.cpp
#include <gtest/gtest.h>
#include <cmath>
#include "../../src/stat_factors/gaussian_copula_factor.h"
#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {

// 测试主题
static const char* TOP_PREDICTION = "gaussian_copula/prediction";

// 时间生成工具
inline int64_t ms_of(int h, int m, int s, int ms) {
    return ((int64_t)h * 3600 + m * 60 + s) * 1000 + ms;
}

class GaussianCopulaFactorTest : public ::testing::Test {
protected:
    void SetUp() override {
        GaussianCopulaFactor::register_topics(2048);

        GaussianCopulaConfig cfg;
        cfg.window_size = 5;
        cfg.debug_mode = false;
        cfg.regularization = 1e-6;

        factor = std::make_unique<GaussianCopulaFactor>(cfg, std::vector<std::string>{"TEST001"});
    }

    void TearDown() override {
        // 手动清理预测数据
        auto& bus = DataBus::instance();
        auto predictions = bus.get_last_n<double>("gaussian_copula/prediction", test_code, 1000);
        // 通过获取所有数据来"清理"它们
        factor.reset();
    }

    std::unique_ptr<GaussianCopulaFactor> factor;
    std::string test_code = "TEST001";

    int64_t ms_of(int hour, int minute, int second, int millisecond = 0) {
        return ((hour * 3600 + minute * 60 + second) * 1000 + millisecond);
    }
};

    // tests/gaussian_copula_factor_test.cpp
    // 在 BasicFunctionality 测试中添加调试信息
    TEST_F(GaussianCopulaFactorTest, BasicFunctionality) {
        auto& bus = DataBus::instance();

        // 生成测试数据 - 增加数据量确保窗口填满
        for (int i = 0; i < 25; ++i) {  // 从15增加到25
            // 生成行情数据
            QuoteDepth q;
            q.instrument_id = test_code;
            q.data_time_ms = ms_of(9, 30, 0, i * 100);
            q.bid_price = 10.0 + i * 0.01;
            q.ask_price = 10.02 + i * 0.01;

            factor->on_quote(q);

            // 生成委托数据（模拟OFI）
            Entrust e;
            e.instrument_id = test_code;
            e.data_time_ms = ms_of(9, 30, 0, i * 100 + 50);  // 在行情之后

            // 交替生成买方和卖方委托
            if (i % 2 == 0) {
                e.side = 1;  // 买方
                e.volume = 100 * (i + 1);
            } else {
                e.side = -1; // 卖方
                e.volume = 80 * (i + 1);
            }

            factor->on_tick(e);
        }

        // 检查是否有预测值发布
        auto predictions = bus.get_last_n<double>(TOP_PREDICTION, test_code, 10);

        // 调试信息
        std::cout << "预测值数量: " << predictions.size() << std::endl;
        for (const auto& pred : predictions) {
            std::cout << "时间: " << pred.first << ", 预测值: " << pred.second << std::endl;
        }

        // 应该有预测值
        EXPECT_GE(predictions.size(), 1u) << "应该至少有一个预测值";

        if (!predictions.empty()) {
            double last_prediction = predictions.back().second;
            std::cout << "最后预测值: " << last_prediction << std::endl;

            // 预测值应该在合理范围内
            EXPECT_TRUE(std::abs(last_prediction) < 0.1) << "预测收益率应该在合理范围内";
        }
    }

    // 测试窗口管理
    TEST_F(GaussianCopulaFactorTest, WindowManagement) {
        auto& bus = DataBus::instance();
        std::string unique_code = "TEST_WINDOW_MGMT";  // 使用唯一代码

        // 为这个测试创建独立的因子实例
        GaussianCopulaConfig cfg;
        cfg.window_size = 5;
        cfg.debug_mode = false;
        cfg.regularization = 1e-6;
        auto test_factor = std::make_unique<GaussianCopulaFactor>(cfg, std::vector<std::string>{unique_code});

        // 喂入少于窗口大小的数据
        for (int i = 0; i < 3; ++i) {
            QuoteDepth q;
            q.instrument_id = unique_code;  // 使用唯一代码
            q.data_time_ms = ms_of(9, 30, 0, i * 100);
            q.bid_price = 10.0 + i * 0.01;
            q.ask_price = 10.02 + i * 0.01;

            test_factor->on_quote(q);

            Entrust e;
            e.instrument_id = unique_code;  // 使用唯一代码
            e.data_time_ms = ms_of(9, 30, 0, i * 100 + 50);

            if (i % 2 == 0) {
                e.side = 1;
                e.volume = 100;
            } else {
                e.side = -1;
                e.volume = 100;
            }

            test_factor->on_tick(e);
        }

        // 窗口未满时不应该有预测值
        auto predictions = bus.get_last_n<double>("gaussian_copula/prediction", unique_code, 10);

        std::cout << "WindowManagement测试 - 预测值数量: " << predictions.size() << std::endl;
        EXPECT_EQ(predictions.size(), 0u) << "窗口未满时不应该有预测值";
    }

    // 测试强制刷新
    TEST_F(GaussianCopulaFactorTest, ForceFlush) {
        // 填充足够的数据
        for (int i = 0; i < 15; ++i) {
            QuoteDepth q;
            q.instrument_id = test_code;
            q.data_time_ms = ms_of(9, 30, 0, i * 100);
            q.bid_price = 10.0 + i * 0.01;
            q.ask_price = 10.02 + i * 0.01;

            factor->on_quote(q);

            Entrust e;
            e.instrument_id = test_code;
            e.data_time_ms = ms_of(9, 30, 0, i * 100 + 50);
            e.side = (i % 2 == 0) ? 1 : -1;
            e.volume = 100;

            factor->on_tick(e);
        }

        // 强制刷新
        bool flushed = factor->force_flush(test_code);
        EXPECT_TRUE(flushed) << "强制刷新应该成功";

        auto& bus = DataBus::instance();
        auto predictions = bus.get_last_n<double>(TOP_PREDICTION, test_code, 10);
        EXPECT_GE(predictions.size(), 1u) << "强制刷新后应该有预测值";
    }

    // 测试数值稳定性
    TEST_F(GaussianCopulaFactorTest, NumericalStability) {
        // 测试极端情况下的数值稳定性
        for (int i = 0; i < 15; ++i) {
            QuoteDepth q;
            q.instrument_id = test_code;
            q.data_time_ms = ms_of(9, 30, 0, i * 100);

            // 使用极端价格测试数值稳定性
            q.bid_price = 1000.0 + (i % 3) * 0.001;
            q.ask_price = 1000.02 + (i % 3) * 0.001;

            factor->on_quote(q);

            Entrust e;
            e.instrument_id = test_code;
            e.data_time_ms = ms_of(9, 30, 0, i * 100 + 50);
            e.side = (i % 2 == 0) ? 1 : -1;
            e.volume = 1000000;  // 大成交量

            factor->on_tick(e);
        }

        auto& bus = DataBus::instance();
        auto predictions = bus.get_last_n<double>(TOP_PREDICTION, test_code, 10);

        // 即使极端数据，也应该能够计算
        EXPECT_GE(predictions.size(), 1u) << "极端数据下也应该有预测值";

        if (!predictions.empty()) {
            double prediction = predictions.back().second;
            EXPECT_FALSE(std::isnan(prediction)) << "预测值不应该为NaN";
            EXPECT_FALSE(std::isinf(prediction)) << "预测值不应该为无穷大";
        }
    }

    TEST_F(GaussianCopulaFactorTest, PerformanceComparison) {
    // 测试大量数据下的性能
    const int num_iterations = 1000;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_iterations; ++i) {
        QuoteDepth q;
        q.instrument_id = test_code;
        q.data_time_ms = ms_of(9, 30, 0, i * 10);
        q.bid_price = 10.0 + (i % 100) * 0.01;
        q.ask_price = 10.02 + (i % 100) * 0.01;

        factor->on_quote(q);

        Entrust e;
        e.instrument_id = test_code;
        e.data_time_ms = ms_of(9, 30, 0, i * 10 + 5);
        e.side = (i % 2 == 0) ? 1 : -1;
        e.volume = 100 + (i % 50);

        factor->on_tick(e);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    LOG_INFO("处理 {} 次迭代耗时: {} ms", num_iterations, duration.count());

    // 验证性能要求：处理1000次迭代应该在合理时间内完成
    EXPECT_LT(duration.count(), 1000) << "增量计算性能应该优于1000ms";
    }

} // namespace