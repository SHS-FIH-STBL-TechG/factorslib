// tests/integration/demo_min_e2e_test.cpp
#include <gtest/gtest.h>
#include "../../demo_header/AppDemo.h"
#include "../../demo_header/DataType.h"
#include "../../factors_lib/include/factorlib/bridge/ingress.h"
#include "../../factors_lib/include/utils/databus.h"
#include "../../factors_lib/src/basic_factors/tick_trans_orders.h"
#include "../../factors_lib/src/gaussian_copula_factor.h"
#include <memory>

using namespace factorlib;
static const char* TOP_PREDICTION = "gaussian_copula/prediction";

inline uint32_t sec_of(int h, int m, int s){ return (uint32_t)(h*3600 + m*60 + s); }
inline int64_t  ms_of (int h, int m, int s, int ms){ return (int64_t)sec_of(h,m,s)*1000 + ms; }

// 造快照（TradeTime 为“秒”）
static std::vector<SnapshotStockSH> make_snaps(uint32_t secbase){
    std::vector<SnapshotStockSH> v;
    SnapshotStockSH s{};
    s.SecurityID = 600000; s.TradeDate = 20251111; s.TradeTime = secbase + 0;
    s.BidPrice01 = 100000; s.OfferPrice01 = 100200;
    s.TotalVolumeTrade = 0; s.TotalValueTrade = 0;
    v.push_back(s);
    s.TradeTime = secbase + 0; s.TotalVolumeTrade = 1000; s.TotalValueTrade = 10000000; v.push_back(s);
    s.TradeTime = secbase + 1; s.BidPrice01 = 100200; s.OfferPrice01 = 100400; s.TotalVolumeTrade = 1800; s.TotalValueTrade = 17000000; v.push_back(s);
    return v;
}

static std::vector<OrdAndExeInfo> make_ont(uint32_t secbase){
    std::vector<OrdAndExeInfo> v;
    OrdAndExeInfo x{}; x.SecurityID=600000; x.BSFlag='B';
    x.TradeTime=secbase+0; x.TradePrice=100100; x.TradeQty=100; v.push_back(x); // 10.01
    x.TradeTime=secbase+1; x.TradePrice=100300; x.TradeQty=50;  v.push_back(x); // 10.03
    return v;
}

TEST(DemoMinE2E, FullPipeline_DemoBridgeFactors)
{
    // 注册两类因子（测试侧）
    auto f1 = std::make_shared<TickTransOrdersFactor>(TickTransOrdersConfig{500, true});
    auto f2 = std::make_shared<GaussianCopulaFactor>(GaussianCopulaConfig{}, std::vector<std::string>{"600000.SH"});
    factorlib::bridge::set_factors({f1, f2});

    AppDemo app;
    ASSERT_EQ(app.Init(), 0);

    auto snaps = make_snaps(sec_of(9,30,0));
    auto ont   = make_ont  (sec_of(9,30,0));
    std::vector<BasicandEnhanceKLine> kline; // 可选，当前留空
    app.RecvSnapStruct(snaps);
    app.RecvOntStruct(ont);
    app.RecvKLineData(kline);

    auto& bus = DataBus::instance();
    std::string code = "600000.SH";
    int64_t ts = 0;

    double mid=0;  ASSERT_TRUE(bus.get_latest<double>("zyd/midprice", code, mid, &ts));
    double amt=0;  ASSERT_TRUE(bus.get_latest<double>("zyd/amount",   code, amt, &ts));
    int64_t vol=0; ASSERT_TRUE(bus.get_latest<int64_t>("zyd/volume",  code, vol, &ts));

    std::vector<Transaction> trans; ASSERT_TRUE(bus.get_latest<std::vector<Transaction>>("zyd/tick/trans", code, trans, &ts));
    std::vector<Entrust> ords;      ASSERT_TRUE(bus.get_latest<std::vector<Entrust>>("zyd/tick/orders",    code, ords,  &ts));

    double pred=0; ASSERT_TRUE(bus.get_latest<double>(TOP_PREDICTION, code, pred, &ts));

    app.Finish();
}
