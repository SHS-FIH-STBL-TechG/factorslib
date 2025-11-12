#include <gtest/gtest.h>
#include <random>
#include <cmath>
#include "granger_causality_factor.h"
#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

/**
 * 合成数据：x 白噪声；y = 0.6*y_{t-1} + 0.8*x_{t-1} + 噪声
 * 逐条事件模式：
 *   - 用 Quote 驱动 y（mid 变动），确保每步有 y_t
 *   - 用 Entrust 累积 x（OFI）
 */
TEST(GrangerCausality, SyntheticCausal_EventDriven) {
    std::vector<std::string> codes = {"000001.SZ"};
    GrangerConfig cfg;
    cfg.window_size = 300;
    cfg.p_lags = 2;
    cfg.q_lags = 1;
    cfg.min_effective = 30;
    cfg.use_neglog10 = true;
    cfg.publish_raw_p = true;
    cfg.debug_mode = false;
    cfg.feed_mode = config::FeedMode::EventDriven; // 明确逐条

    GrangerCausalityFactor fac(codes, cfg);
    fac.register_topics(2048);
    fac.on_code_added(codes[0]);

    std::mt19937 rng(42);
    std::normal_distribution<double> n01(0.0, 1.0);

    const int T = 400;
    double last_mid = 100.0;
    double y_prev = 0.0;
    double x_prev = 0.0;

    for (int t=0; t<T; ++t) {
        double x = n01(rng);
        double eps = 0.5*n01(rng);
        double y = 0.6*y_prev + 0.8*x_prev + eps;

        // 先用 entrust 累积 OFI（买-卖 ≈ x*100）
        Entrust e_buy;  e_buy.instrument_id=codes[0]; e_buy.data_time_ms=t*2; e_buy.side=+1; e_buy.volume = (uint64_t) std::max(0.0, 50 + 50*x);
        Entrust e_sell; e_sell.instrument_id=codes[0]; e_sell.data_time_ms=t*2+1; e_sell.side=-1; e_sell.volume = (uint64_t) std::max(0.0, 50 - 50*x);
        fac.on_entrust(e_buy);
        fac.on_entrust(e_sell);

        // 用 quote 触发 y_t（mid 变化）
        QuoteDepth q;
        q.instrument_id = codes[0];
        q.data_time_ms  = t*2+2;
        q.bid_price = last_mid + 0.5*y;
        q.ask_price = last_mid + 1.5*y;
        last_mid = 0.5*(q.bid_price + q.ask_price);
        fac.on_quote(q);

        y_prev = y; x_prev = x;
    }

    double strength=0, pval=1; int64_t ts=0;
    ASSERT_TRUE( DataBus::instance().get_latest<double>(TOP_GRANGER_STRENGTH, codes[0], strength, &ts) );
    ASSERT_TRUE( DataBus::instance().get_latest<double>(TOP_GRANGER_PVAL,     codes[0], pval, &ts) );
    EXPECT_LT(pval, 1e-3);
    EXPECT_GT(strength, 3.0); // -log10(1e-3)=3
}
