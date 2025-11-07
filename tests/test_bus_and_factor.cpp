
#include <gtest/gtest.h>
#include "basic_factors/tick_trans_orders.h"
#include "utils/databus.h"
#include "utils/log.h"
#include "utils/utils.h"
#include "utils/data_gen.h"

/**
 * @file test_bus_and_factor.cpp
 * @brief 用例目标：
 *  1) 验证模板化 DataBus 的基本读写与类型安全；
 *  2) 验证 0109 翻译因子的 per-bucket 产出是否与手工期望一致。
 */

using namespace factorlib;
using namespace factorlib::testutil;

TEST(DataBus, TemplateChannelBasicIO){
    // 本测试演示“模板化”主题绑定：demo/price 绑定为 double
    DataBus::instance().register_topic<double>("demo/price", 4);
    DataBus::instance().publish<double>("demo/price", "000001.SZ", 1000, 10.5);
    double v=0; int64_t ts=0;
    ASSERT_TRUE(DataBus::instance().get_latest<double>("demo/price","000001.SZ", v, &ts));
    EXPECT_DOUBLE_EQ(v, 10.5);
    EXPECT_EQ(ts, 1000);
}

TEST(TickTransOrders, PerBucketOutputsMatchExpectations){
    // 注册 0109 对应的五类 topic
    TickTransOrders::register_topics(8);
    TickTransOrdersConfig cfg; cfg.bucket_size_ms = 1000; // 1s 桶
    TickTransOrders factor(cfg, {"000001.SZ"});
    const std::string code = "000001.SZ";
    auto s = make_series_basic(code, hms_ms(9,30,0), cfg.bucket_size_ms);

    // 1) 依次喂交易、委托、行情（顺序不严格要求，但本测试按时间顺序）
    for (auto& t : s.trans) factor.on_transaction(t);
    for (auto& e : s.orders) factor.on_entrust(e);
    for (auto& q : s.quotes) factor.on_quote(q);

    // 2) 触发第一个桶产出：将 now 推进到 9:30:01.000
    factor.on_quote(QuoteDepth{code, hms_ms(9,30,1,0), 20250101, 2000, 200000.0, 10.0, 10.2});
    int64_t ts_b1 = hms_ms(9,30,1,0);

    // 3) 读取并断言 amount/volume/midprice
    double amt=0, mid=0; int64_t vol=0;
    ASSERT_TRUE(DataBus::instance().get_by_time_exact<double>("zyd/amount", code, ts_b1, amt));
    ASSERT_TRUE(DataBus::instance().get_by_time_exact<int64_t>("zyd/volume", code, ts_b1, vol));
    ASSERT_TRUE(DataBus::instance().get_by_time_exact<double>("zyd/midprice", code, ts_b1, mid));
    EXPECT_NEAR(amt, 40000.0, 1e-9);    // 第一桶累计额增量之和
    EXPECT_EQ(vol, 400);                // 第一桶累计量增量之和
    EXPECT_NEAR(mid, (10.0+10.2)/2.0, 1e-9);  // 第一桶最后一笔中价

    // 4) 切片断言：第一桶内有 2 笔成交与 1 笔委托
    std::vector<Transaction> vt; std::vector<Entrust> vo;
    ASSERT_TRUE(DataBus::instance().get_by_time_exact<std::vector<Transaction>>("zyd/tick/trans", code, ts_b1, vt));
    ASSERT_TRUE(DataBus::instance().get_by_time_exact<std::vector<Entrust>>("zyd/tick/orders", code, ts_b1, vo));
    EXPECT_EQ(vt.size(), 2u);
    EXPECT_EQ(vo.size(), 1u);

    // 5) 触发第二桶产出，并断言增量
    factor.on_quote(QuoteDepth{code, hms_ms(9,30,2,0), 20250101, 2000, 200000.0, 10.0, 10.2});
    int64_t ts_b2 = hms_ms(9,30,2,0);
    ASSERT_TRUE(DataBus::instance().get_by_time_exact<double>("zyd/amount", code, ts_b2, amt));
    ASSERT_TRUE(DataBus::instance().get_by_time_exact<int64_t>("zyd/volume", code, ts_b2, vol));
    EXPECT_NEAR(amt, 60000.0, 1e-9);   // 第二桶：20000 + 40000 = 60000
    EXPECT_EQ(vol, 600);               // 第二桶：200 + 400 = 600
}
