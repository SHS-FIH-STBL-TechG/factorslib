// tests/gaussian_copula_factor_test.cpp
#include <gtest/gtest.h>
#include <cmath>
#include "gaussian_copula_factor.h"
#include "utils/databus.h"
#include "utils/utils.h"

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
        cfg.window_size = 10;  // 测试使用较小窗口
        cfg.debug_mode = true;
        
        factor = std::make_unique<GaussianCopulaFactor>(cfg, std::vector<std::string>{"TEST001"});
    }

    void TearDown() override {
        factor.reset();
        DataBus::instance().reset();
    }

    std::unique_ptr<GaussianCopulaFactor> factor;
    std::string test_code = "TEST001";
};

// 测试基础功能
TEST_F(GaussianCopulaFactorTest, BasicFunctionality) {
    auto& bus = DataBus::instance();
    
    // 生成测试数据
    for (int i = 0; i < 15; ++i) {
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
        
        factor->on_entrust(e);
    }
    
    // 检查是否有预测值发布
    auto predictions = bus.get_last_n<double>(TOP_PREDICTION, test_code, 10);
    
    // 应该有5个预测值（前10个tick填充窗口，后5个产生预测）
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
    // 填充少于窗口大小的数据
    for (int i = 0; i < 5; ++i) {
        QuoteDepth q;
        q.instrument_id = test_code;
        q.data_time_ms = ms_of(9, 30, 0, i * 100);
        q.bid_price = 10.0;
        q.ask_price = 10.02;
        
        factor->on_quote(q);
        
        Entrust e;
        e.instrument_id = test_code;
        e.data_time_ms = ms_of(9, 30, 0, i * 100 + 50);
        e.side = 1;
        e.volume = 100;
        
        factor->on_entrust(e);
    }
    
    auto& bus = DataBus::instance();
    auto predictions = bus.get_last_n<double>(TOP_PREDICTION, test_code, 10);
    
    // 窗口未满，不应该有预测值
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
        
        factor->on_entrust(e);
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
        
        factor->on_entrust(e);
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

} // namespace