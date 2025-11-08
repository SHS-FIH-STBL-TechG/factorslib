// tests/tick_trans_orders_test.cpp
//
// 覆盖点：
// 1) 桶级 5 个主题：
//    - zyd/amount   (double)
//    - zyd/volume   (int64_t)
//    - zyd/midprice (double)
//    - zyd/tick/trans   (std::vector<Transaction>)
//    - zyd/tick/orders  (std::vector<Entrust>)
// 2) 两个 tick 之间的产出（暂不支持，以下断言已注释）
//    - zyd/interval/trans
//    - zyd/interval/orders
//
// 重要说明：
// - 按你要求，amount/volume/midprice 不做“两个 tick 之间”的产出（本测试也不做此类断言）。
// - 这里假设 1107 分支的 DataBus 暴露了 register_topic<T> / get_all<T> / get_latest<T> 等方法；
//   如你工程 API 名字或签名略有不同，请在本文件中把相应调用替换成你们现有的读取方式。
// - TickTransOrders 的改造（增加 interval 输出）已合入；若主题名不同，请同步下面的常量。

#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "basic_factors/tick_trans_orders.h"
#include "utils/databus.h"
#include "utils/utils.h"

using namespace factorlib;

namespace {

// 统一的主题名（与源码保持一致）
static const char* TOP_AMOUNT  = "zyd/amount";
static const char* TOP_VOLUME  = "zyd/volume";
static const char* TOP_MID     = "zyd/midprice";
static const char* TOP_TTRANS  = "zyd/tick/trans";
static const char* TOP_TORD    = "zyd/tick/orders";
static const char* TOP_ITRANS  = "zyd/interval/trans";
static const char* TOP_IORD    = "zyd/interval/orders";

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
    factor->on_transaction(t1);

    Transaction t2{};
    t2.instrument_id = code;
    t2.data_time_ms  = ms_of(9,30,0,150);  // 后发生
    t2.main_seq      = 2;
    t2.price=10.01; t2.side=-1; t2.volume=5; t2.bid_no=3; t2.ask_no=4;
    factor->on_transaction(t2);

    Entrust e1{};
    e1.instrument_id = code;
    e1.data_time_ms  = ms_of(9,30,0,130);
    e1.main_seq      = 7;
    e1.price=10.01; e1.side=-1; e1.volume=5; e1.order_id=11111;
    factor->on_entrust(e1);

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
    bus.debug_print_topics();

    // 第一个桶内的数据
    QuoteDepth q{};
    q.instrument_id = code;
    q.trading_day = 20250101;
    q.data_time_ms = ms_of(9,30,0,100); // 34200100
    q.volume = 1000;
    q.turnover = 10000.0;
    q.bid_price = 10.00;
    q.ask_price = 10.02;

    std::cout << "[测试] 喂入第一个tick:" << std::endl;
    std::cout << "  时间: " << q.data_time_ms << std::endl;
    std::cout << "  volume: " << q.volume << " (累计)" << std::endl;
    std::cout << "  turnover: " << q.turnover << " (累计)" << std::endl;
    std::cout << "  买卖价: " << q.bid_price << "/" << q.ask_price << std::endl;

    factor->on_quote(q);

    // 第二个tick，跨越第一个桶边界
    q.data_time_ms = ms_of(9,30,0,600); // 34200600
    q.volume = 1500;    // +500
    q.turnover = 15500.0; // +5500
    q.bid_price = 10.01;
    q.ask_price = 10.03;

    std::cout << "[测试] 喂入第二个tick:" << std::endl;
    std::cout << "  时间: " << q.data_time_ms << " (跨越桶边界)" << std::endl;
    std::cout << "  volume: " << q.volume << " (+" << (q.volume - 1000) << ")" << std::endl;
    std::cout << "  turnover: " << q.turnover << " (+" << (q.turnover - 10000.0) << ")" << std::endl;
    std::cout << "  买卖价: " << q.bid_price << "/" << q.ask_price << std::endl;

    factor->on_quote(q);

    // 第三个tick，在第二个桶内
    q.data_time_ms = ms_of(9,30,0,700); // 34200700
    q.volume = 1800;    // +300
    q.turnover = 17000.0; // +1500
    q.bid_price = 10.02;
    q.ask_price = 10.04;

    std::cout << "[测试] 喂入第三个tick:" << std::endl;
    std::cout << "  时间: " << q.data_time_ms << " (在第二个桶内)" << std::endl;
    std::cout << "  volume: " << q.volume << " (+" << (q.volume - 1500) << ")" << std::endl;
    std::cout << "  turnover: " << q.turnover << " (+" << (q.turnover - 15500.0) << ")" << std::endl;
    std::cout << "  买卖价: " << q.bid_price << "/" << q.ask_price << std::endl;

    factor->on_quote(q);

    // 强制刷新当前桶，确保第二个桶的数据被发布
    factor->force_flush(code);

    // 最终检查
    bus.debug_print_topics();

    auto amounts = bus.get_last_n<double>(TOP_AMOUNT, code, 10);
    auto volumes = bus.get_last_n<int64_t>(TOP_VOLUME, code, 10);
    auto midprices = bus.get_last_n<double>(TOP_MID, code, 10);

    std::cout << "[结果] 获取到的数据条数:" << std::endl;
    std::cout << "  amounts: " << amounts.size() << std::endl;
    std::cout << "  volumes: " << volumes.size() << std::endl;
    std::cout << "  midprices: " << midprices.size() << std::endl;

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