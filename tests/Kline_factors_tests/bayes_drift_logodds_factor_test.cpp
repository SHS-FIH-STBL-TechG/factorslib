#include "factors/kline/bayes_drift_logodds_factor.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace factorlib {

class TestBayesDriftLogOddsFactor : public BayesDriftLogOddsFactor {
public:
    using BayesDriftLogOddsFactor::BayesDriftLogOddsFactor;
    void on_quote(const QuoteDepth&) override {}
    void on_tick(const CombinedTick&) override {}
};

static std::vector<Bar> makeTrendBars(const std::string& code, int n, double start_price, double step) {
    std::vector<Bar> bars;
    bars.reserve(static_cast<std::size_t>(n));
    int64_t t0 = 3000000;
    double price = start_price;
    for (int i = 0; i < n; ++i) {
        Bar b;
        b.instrument_id = code;
        b.data_time_ms = t0 + i * 60000;
        b.close = price;
        b.open = price;
        b.high = price * 1.01;
        b.low = price * 0.99;
        b.volume = 1000;
        b.turnover = price * 1000;
        b.interval_ms = 60000;
        bars.push_back(b);
        price = std::max(1e-6, price + step);
    }
    return bars;
}

TEST(BayesDriftLogOddsFactorTest, UpDownSign) {
    DataBus::instance().reset();
    TestBayesDriftLogOddsFactor::register_topics(256);

    TestBayesDriftLogOddsFactor up({"000001.SZ"});
    auto upBars = makeTrendBars("000001.SZ", 30, 100.0, 1.0);
    for (const auto& b : upBars) up.on_bar(b);
    double v_up = 0.0;
    EXPECT_TRUE(DataBus::instance().get_latest<double>("kline/bayes_drift_logodds", "000001.SZ", v_up));
    EXPECT_TRUE(std::isfinite(v_up));
    EXPECT_GT(v_up, 0.0);

    DataBus::instance().reset();
    TestBayesDriftLogOddsFactor::register_topics(256);

    TestBayesDriftLogOddsFactor down({"000002.SZ"});
    auto downBars = makeTrendBars("000002.SZ", 30, 100.0, -1.0);
    for (const auto& b : downBars) down.on_bar(b);
    double v_down = 0.0;
    EXPECT_TRUE(DataBus::instance().get_latest<double>("kline/bayes_drift_logodds", "000002.SZ", v_down));
    EXPECT_TRUE(std::isfinite(v_down));
    EXPECT_LT(v_down, 0.0);
}

} // namespace factorlib

