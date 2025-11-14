// tests/tick_trans_orders_test.cpp
//
// 覆盖点：
// 1) 桶级 5 个主题：
//    - zyd/amount   (double)
//    - zyd/volume   (int64_t)
//    - zyd/midprice (double)
//    - zyd/tick/trans   (std::vector<Transaction>)
//    - zyd/tick/orders  (std::vector<Entrust>)
// 2) 两个 tick 之间的产出
//    - zyd/interval/trans
//    - zyd/interval/orders

#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "basic_factors/tick_trans_orders.h"
#include "utils/databus.h"
#include "utils/types.h"
#include "../utils/test_config.h"

using namespace factorlib;

namespace {

// 统一的主题名（与源码保持一致）
static const char* TOP_AMOUNT  = "zyd/amount";    // 桶级成交额主题 - 每个时间桶内的成交额增量
static const char* TOP_VOLUME  = "zyd/volume";    // 桶级成交量主题 - 每个时间桶内的成交量增量
static const char* TOP_MID     = "zyd/midprice";  // 桶级中价主题 - 每个时间桶内最后一个tick的中价
static const char* TOP_TTRANS  = "zyd/tick/trans";  // 桶级成交切片主题 - 每个时间桶内的所有成交记录
static const char* TOP_TORD    = "zyd/tick/orders"; // 桶级委托切片主题 - 每个时间桶内的所有委托记录
static const char* TOP_ITRANS  = "zyd/interval/trans";  // interval级成交主题 - 两个相邻tick之间的成交记录
static const char* TOP_IORD    = "zyd/interval/orders"; // interval级委托主题 - 两个相邻tick之间的委托记录

// 用于构造“当日毫秒”时间
inline int64_t ms_of(int h,int m,int s,int ms){
    return ((int64_t)h*3600 + m*60 + s)*1000 + ms;
}

} // namespace

// =============== 测试夹具（可复用） ===============
class TickTransOrdersFixture : public ::testing::Test {
protected:
    void SetUp() override {
        TickTransOrders::register_topics(2048);
        factor = std::make_unique<TickTransOrders>(cfg, std::vector<std::string>{code});
    }

    void TearDown() override {
        factor.reset();
        DataBus::instance().reset();  // 重置数据总线
    }

    TickTransOrdersConfig cfg{500, true};
    std::unique_ptr<TickTransOrders> factor;
    std::string code{"SH600000"};
};

// =============== 用例 1：两个 tick 之间的切片产出 ===============
TEST_F(TickTransOrdersFixture, IntervalBetweenTicks_TransAndOrdersOnly)
{
    auto& bus = DataBus::instance();

    // tick1：09:30:00.000
    QuoteDepth q{};
    q.instrument_id = code;
    q.trading_day   = 20250101;
    q.data_time_ms  = ms_of(9,30,0,0);
    q.volume        = 100;
    q.turnover      = 1000.0;
    q.bid_price     = 9.99;
    q.ask_price     = 10.01;
    factor->on_quote(q);

    // (tick1, tick2] 内的成交/委托（按时间顺序）
    Transaction t1{};
    t1.instrument_id = code;
    t1.data_time_ms  = ms_of(9,30,0,120);  // 先发生
    t1.main_seq      = 1;
    t1.price=10.00; t1.side=1; t1.volume=10; t1.bid_no=1; t1.ask_no=2;
    factor->on_tick(t1);

    Transaction t2{};
    t2.instrument_id = code;
    t2.data_time_ms  = ms_of(9,30,0,150);  // 后发生
    t2.main_seq      = 2;
    t2.price=10.01; t2.side=-1; t2.volume=5; t2.bid_no=3; t2.ask_no=4;
    factor->on_tick(t2);

    Entrust e1{};
    e1.instrument_id = code;
    e1.data_time_ms  = ms_of(9,30,0,130);
    e1.main_seq      = 7;
    e1.price=10.01; e1.side=-1; e1.volume=5; e1.order_id=11111;
    factor->on_tick(e1);

    // tick2：09:30:00.500 —— 到达"下一个tick"，应发布(tick1, tick2]区间产出
    q.data_time_ms  = ms_of(9,30,0,500);
    q.volume        = 140;       // +40
    q.turnover      = 1400.0;    // +400
    q.bid_price     = 10.00;
    q.ask_price     = 10.02;
    factor->on_quote(q);

    // 读取 interval 主题，校验 (tick1, tick2] 的内容
    std::vector<std::pair<int64_t, std::vector<Transaction>>> ivt =
        bus.get_last_n<std::vector<Transaction>>(TOP_INTERVAL_TRANS, code, 10);
    std::vector<std::pair<int64_t, std::vector<Entrust>>> ivo =
        bus.get_last_n<std::vector<Entrust>>(TOP_INTERVAL_ORDERS, code, 10);

    ASSERT_FALSE(ivt.empty()) << "No interval transaction data published";
    ASSERT_FALSE(ivo.empty()) << "No interval order data published";

    auto iv_trans = ivt.back();
    auto iv_orders = ivo.back();

    // 时间戳应等于 tick2 的时间
    EXPECT_EQ(iv_trans.first, ms_of(9,30,0,500));
    EXPECT_EQ(iv_orders.first, ms_of(9,30,0,500));

    // (tick1, tick2] 期间应该有 2 笔成交 + 1 笔委托
    ASSERT_EQ(iv_trans.second.size(), 2u) << "Should have 2 transactions in interval";
    ASSERT_EQ(iv_orders.second.size(), 1u) << "Should have 1 order in interval";

    // 检查成交的顺序（按喂入顺序，不是按时间排序）
    EXPECT_EQ(iv_trans.second[0].main_seq, 1u);  // 先喂入的 t1
    EXPECT_EQ(iv_trans.second[1].main_seq, 2u);  // 后喂入的 t2
    EXPECT_EQ(iv_orders.second[0].main_seq, 7u); // 委托
}

TEST_F(TickTransOrdersFixture, BucketAggregation_SimpleTest)
{
    auto& bus = DataBus::instance();

    // 简单的测试：只验证数据是否被发布
    QuoteDepth q{};
    q.instrument_id = code;
    q.data_time_ms = ms_of(9,30,0,100);
    q.volume = 1000;
    q.turnover = 10000.0;
    q.bid_price = 10.00;
    q.ask_price = 10.02;
    factor->on_quote(q);

    // 强制刷新当前桶
    factor->force_flush(code);

    auto amounts = bus.get_last_n<double>(TOP_AMOUNT, code, 10);
    ASSERT_FALSE(amounts.empty()) << "强制刷新后应该有数据";
}

// =============== 用例 2：验证桶聚合功能 ===============
TEST_F(TickTransOrdersFixture, BucketAggregation_BasicFunctionality)
{
    auto& bus = DataBus::instance();

    std::cout << "=== 开始桶聚合测试 ===" << std::endl;

    // 第一个桶内的数据
    QuoteDepth q{};
    q.instrument_id = code;
    q.trading_day = 20250101;
    q.data_time_ms = ms_of(9,30,0,100); // 34200100
    q.volume = 1000;
    q.turnover = 10000.0;
    q.bid_price = 10.00;
    q.ask_price = 10.02;

    factor->on_quote(q);

    // 第二个tick，跨越第一个桶边界
    q.data_time_ms = ms_of(9,30,0,600); // 34200600
    q.volume = 1500;    // +500
    q.turnover = 15500.0; // +5500
    q.bid_price = 10.01;
    q.ask_price = 10.03;

    factor->on_quote(q);

    // 第三个tick，在第二个桶内
    q.data_time_ms = ms_of(9,30,0,700); // 34200700
    q.volume = 1800;    // +300
    q.turnover = 17000.0; // +1500
    q.bid_price = 10.02;
    q.ask_price = 10.04;

    factor->on_quote(q);

    // 强制刷新当前桶，确保第二个桶的数据被发布
    factor->force_flush(code);

    auto amounts = bus.get_last_n<double>(TOP_AMOUNT, code, 10);
    auto volumes = bus.get_last_n<int64_t>(TOP_VOLUME, code, 10);
    auto midprices = bus.get_last_n<double>(TOP_MID, code, 10);

    // 应该有两个桶的数据
    ASSERT_EQ(amounts.size(), 2u) << "应该有两个桶的amount数据";
    ASSERT_EQ(volumes.size(), 2u) << "应该有两个桶的volume数据";
    ASSERT_EQ(midprices.size(), 2u) << "应该有两个桶的midprice数据";

    // 第一个桶的数据（34200000-34200500）
    EXPECT_EQ(amounts[0].first, ms_of(9,30,0,500));
    EXPECT_DOUBLE_EQ(amounts[0].second, 10000.0);  // 相对于0的增量
    EXPECT_EQ(volumes[0].first, ms_of(9,30,0,500));
    EXPECT_EQ(volumes[0].second, 1000);           // 相对于0的增量
    EXPECT_DOUBLE_EQ(midprices[0].second, 10.01); // 第一个tick的中价 (10.00+10.02)/2

    // 第二个桶的数据（34200500-34201000）
    EXPECT_EQ(amounts[1].first, ms_of(9,30,1,0));
    EXPECT_DOUBLE_EQ(amounts[1].second, 7000.0);  // 相对于第一个tick的增量 (5500+1500)
    EXPECT_EQ(volumes[1].first, ms_of(9,30,1,0));
    EXPECT_EQ(volumes[1].second, 800);            // 相对于第一个tick的增量 (500+300)
    EXPECT_DOUBLE_EQ(midprices[1].second, 10.03); // 最后一个tick的中价 (10.02+10.04)/2
}

// ============== 用例：CSV 驱动桶聚合冒烟测试 ==============
TEST_F(TickTransOrdersFixture, BucketAggregation_CsvFeed_Smoke) {
    auto& bus = DataBus::instance();

    auto quotes = testcfg::read_quotes_from_cfg();
    auto trans  = testcfg::read_transactions_from_cfg();

    if (quotes.empty() || trans.empty()) {
        GTEST_SKIP() << "quotes_csv / transactions_csv 未配置或为空";
    }

    size_t n = std::min(quotes.size(), trans.size());
    ASSERT_GE(n, 5u);

    for (size_t i = 0; i < n; ++i) {
        auto q = quotes[i];
        q.instrument_id = code;
        factor->on_quote(q);

        auto t = trans[i];
        t.instrument_id = code;
        factor->on_tick(t);   // Transaction

        Entrust e{};
        e.instrument_id = code;
        e.data_time_ms  = t.data_time_ms;
        e.main_seq      = t.main_seq;
        e.price         = t.price;
        e.side          = t.side;
        e.volume        = t.volume;
        e.order_id      = t.main_seq;

        factor->on_tick(e);   // Entrust
    }

    // 检查桶级产出是否存在
    auto amount = bus.get_last_n<double>(TOP_AMOUNT, code, 10);
    auto volume = bus.get_last_n<int64_t>(TOP_VOLUME, code, 10);
    auto mid    = bus.get_last_n<double>(TOP_MID, code, 10);
    auto ttrans = bus.get_last_n<std::vector<Transaction>>(TOP_TTRANS, code, 10);
    auto tord   = bus.get_last_n<std::vector<Entrust>>(TOP_TORD, code, 10);

    ASSERT_FALSE(amount.empty()) << "CSV 喂数后应有金额聚合产出";
    ASSERT_FALSE(volume.empty()) << "CSV 喂数后应有成交量聚合产出";
    ASSERT_FALSE(mid.empty())    << "CSV 喂数后应有中间价聚合产出";
    ASSERT_FALSE(ttrans.empty()) << "CSV 喂数后应有逐笔成交聚合产出";

    // interval 两个主题在 fixture 配置中 emit_tick_interval=true
    auto ivt = bus.get_last_n<std::vector<Transaction>>(TOP_INTERVAL_TRANS, code, 10);
    auto ivo = bus.get_last_n<std::vector<Entrust>>(TOP_INTERVAL_ORDERS, code, 10);

    // 因为我们喂了多条 quote/tick，至少应有一条 interval 切片
    EXPECT_FALSE(ivt.empty());
    EXPECT_FALSE(ivo.empty());
}
