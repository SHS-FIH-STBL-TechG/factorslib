// factors_lib/tests/integration/demo_min_e2e_test.cpp

#include "gtest/gtest.h"

// 只依赖 demo 的数据结构（用于构造输入）
#include "DataType.h"

// 因子库公共接口
#include "factorlib/bridge/ingress.h"
#include "utils/databus.h"

// 因子实现（与 IFactor 体系）
#include "basic_factors/tick_trans_orders.h"   // 类：factorlib::TickTransOrders
#include "gaussian_copula_factor.h"            // 类：factorlib::GaussianCopulaFactor

#include <memory>

using namespace factorlib;

// ====== 构造输入工具 ======

static SnapshotStockSH mk_snap(uint32_t security_id,
                               int trade_date,        // 例如 20240101
                               int trade_time_sec,    // 秒（适配层会 *1000）
                               int bid_px01,
                               int offer_px01,
                               uint64_t total_vol,    // 成交总量（累计）
                               double   total_amt)    // 成交总额（累计）
{
    SnapshotStockSH s{};
    s.SecurityID        = security_id;
    s.TradeDate         = trade_date;
    s.TradeTime         = trade_time_sec;
    s.BidPrice01        = bid_px01;
    s.OfferPrice01      = offer_px01;
    s.TotalVolumeTrade  = total_vol;
    s.TotalValueTrade   = total_amt;
    return s;
}

static OrdAndExeInfo mk_ont(uint32_t security_id,
                            int trade_time_sec,   // 秒
                            int trade_price,
                            int trade_qty,
                            char bs_flag)         // 'B' 或 'S'
{
    OrdAndExeInfo x{};
    x.SecurityID  = security_id;
    x.TradeTime   = trade_time_sec;
    x.TradePrice  = trade_price;
    x.TradeQty    = trade_qty;
    x.BSFlag      = bs_flag;      // 'B' 买, 'S' 卖
    return x;
}

TEST(DemoMinE2E, FullPipeline_DemoBridgeFactors)
{
    // 与 DataAdapter::security_id_to_string 一致：%06u → "600000"
    const uint32_t secid = 600000;
    const std::string code_key = "600000";

    // 0) 显式注册两个因子的输出主题（很关键）
    TickTransOrders::register_topics(1024);
    GaussianCopulaFactor::register_topics(1024);

    // 1) 注册因子到桥接层
    auto f1 = std::make_shared<TickTransOrders>(
        TickTransOrdersConfig{/*bucket_size_ms=*/500, /*emit_tick_interval=*/true},
        std::vector<std::string>{code_key}
    );
    auto f2 = std::make_shared<GaussianCopulaFactor>(
        GaussianCopulaConfig{},
        std::vector<std::string>{code_key}
    );

    std::vector<std::shared_ptr<IFactor>> fs;
    fs.emplace_back(std::static_pointer_cast<IFactor>(f1));
    fs.emplace_back(std::static_pointer_cast<IFactor>(f2));
    bridge::set_factors(fs);

    // 2) 喂快照（提供 Bid/Ask 用于 midprice；累计量用于 amount/volume）
    {
        std::vector<SnapshotStockSH> snaps;
        snaps.push_back(mk_snap(secid, 20240101, 9*3600+30*60,    135000, 135020, 100000, 1.0e9));
        snaps.push_back(mk_snap(secid, 20240101, 9*3600+30*60+1,  135010, 135030, 100500, 1.005e9));
        bridge::ingest_snapshot(snaps);
    }

    // 3) 喂逐笔（OrdAndExeInfo → Entrust/Transaction，经 data_adapter 转换）
    {
        std::vector<OrdAndExeInfo> onts;
        onts.push_back(mk_ont(secid, 9*3600+30*60+2, 135050, 100, 'B'));
        onts.push_back(mk_ont(secid, 9*3600+30*60+3, 135000, 200, 'S'));
        bridge::ingest_ont(onts);
    }

    // 4) 断言 DataBus 有产出
    auto& bus = DataBus::instance();
    int64_t ts = 0;

    double mid=0;   ASSERT_TRUE(bus.get_latest<double>("zyd/midprice", code_key, mid, &ts));
    double amt=0;   ASSERT_TRUE(bus.get_latest<double>("zyd/amount",   code_key, amt, &ts));
    int64_t vol=0;  ASSERT_TRUE(bus.get_latest<int64_t>("zyd/volume",  code_key, vol, &ts));

    std::vector<Transaction> trans;
    ASSERT_TRUE(bus.get_latest<std::vector<Transaction>>("zyd/tick/trans", code_key, trans, &ts));

    std::vector<Entrust> ords;
    ASSERT_TRUE(bus.get_latest<std::vector<Entrust>>("zyd/tick/orders", code_key, ords, &ts));

    // 5) Gaussian 预测：窗口可能未满，不强制断言
    double pred=0;
    (void)bus.get_latest<double>("gaussian_copula/prediction", code_key, pred, &ts);
}
