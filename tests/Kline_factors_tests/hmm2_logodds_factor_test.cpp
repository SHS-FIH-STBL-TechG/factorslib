#include "factors/kline/hmm2_logodds_factor.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace factorlib {

class TestHmm2LogOddsFactor : public Hmm2LogOddsFactor {
public:
    using Hmm2LogOddsFactor::Hmm2LogOddsFactor;
    void on_quote(const QuoteDepth&) override {}
    void on_tick(const CombinedTick&) override {}
};

static std::vector<Bar> makeTrendBars(const std::string& code, int n, double start_price, double step) {
    std::vector<Bar> bars;
    bars.reserve(static_cast<std::size_t>(n));
    int64_t t0 = 2000000;
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

TEST(Hmm2LogOddsFactorTest, ProducesFiniteAfterWarmup) {
    DataBus::instance().reset();
    TestHmm2LogOddsFactor::register_topics(256);

    Hmm2LogOddsConfig cfg;
    cfg.window_size = 60;
    TestHmm2LogOddsFactor f({"000001.SZ"}, cfg);
    auto bars = makeTrendBars("000001.SZ", 70, 100.0, 1.0);
    for (const auto& b : bars) f.on_bar(b);

    double v = 0.0;
    EXPECT_TRUE(DataBus::instance().get_latest<double>("kline/hmm2_logodds", "000001.SZ", v));
    EXPECT_TRUE(std::isfinite(v));
    EXPECT_GT(v, 0.0);
}

} // namespace factorlib
