// tests/stat_factors_tests/least_squares_info_gain_factor_test.cpp

#include "../../src/stat_factors/least_squares_info_gain_factor.h"

#include <gtest/gtest.h>

#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {

inline int64_t ms_of(int h, int m, int s, int ms = 0) {
    return (((static_cast<int64_t>(h) * 60 + m) * 60) + s) * 1000 + ms;
}

// 每个用例前后清理 DataBus，避免状态串扰
struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

// 取某个 code 的最新 IG 值
bool last_ig(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_LSIG_IG, code, out, ts);
}

std::vector<double> generate_price_series(size_t count, double start_price = 100.0) {
    std::vector<double> prices;
    prices.reserve(count);
    double px = start_price;
    for (size_t i = 0; i < count; ++i) {
        double ret = 0.0006 + 0.0004 * std::sin(0.15 * static_cast<double>(i));
        px *= std::exp(ret);
        prices.push_back(px);
    }
    return prices;
}

std::vector<Bar> make_bars(const std::string& code, size_t count) {
    auto prices = generate_price_series(count);
    std::vector<Bar> bars;
    bars.reserve(count);
    int64_t base_ts = ms_of(9, 30, 0, 0);
    for (size_t i = 0; i < count; ++i) {
        Bar b{};
        b.instrument_id = code;
        b.data_time_ms  = base_ts + static_cast<int64_t>(i) * 60'000;
        b.close         = prices[i];
        b.volume        = 1000 + static_cast<int64_t>(i) * 20;
        bars.push_back(b);
    }
    return bars;
}

std::vector<Transaction> make_transactions(const std::string& code, size_t count) {
    auto prices = generate_price_series(count);
    std::vector<Transaction> trans;
    trans.reserve(count);
    int64_t base_ts = ms_of(9, 30, 0, 500);
    for (size_t i = 0; i < count; ++i) {
        Transaction t{};
        t.instrument_id = code;
        t.data_time_ms  = base_ts + static_cast<int64_t>(i) * 5'000;
        t.main_seq      = static_cast<uint64_t>(i + 1);
        double jitter   = 1.0 + 0.0002 * ((i % 3) - 1);
        t.price         = prices[i] * jitter;
        t.side          = (i % 2 == 0) ? 1 : -1;
        t.volume        = 50 + static_cast<uint64_t>(i % 7) * 10;
        trans.push_back(t);
    }
    return trans;
}

std::vector<QuoteDepth> make_quotes(const std::string& code, size_t count) {
    auto prices = generate_price_series(count);
    std::vector<QuoteDepth> quotes;
    quotes.reserve(count);
    int64_t base_ts = ms_of(9, 30, 0, 250);
    for (size_t i = 0; i < count; ++i) {
        QuoteDepth q{};
        q.instrument_id = code;
        q.data_time_ms  = base_ts + static_cast<int64_t>(i) * 3'000;
        q.bid_price     = prices[i] - 0.01;
        q.ask_price     = prices[i] + 0.01;
        quotes.push_back(q);
    }
    return quotes;
}

} // namespace

// ============== 用例 1：Bar 驱动（合成数据） ==============
TEST(LeastSquaresInfoGain, BarsSyntheticFeedProducesFactor) {
    BusGuard _guard;
    LeastSquaresInfoGainFactor::register_topics(2048);

    auto bars = make_bars("LSIG_BAR", 120);

    const std::string code = "LSIG_BAR";

    LsInfoGainConfig cfg;
    LeastSquaresInfoGainFactor factor({ code }, cfg);

    for (const auto& b : bars) {
        factor.on_bar(b);
    }

    double ig = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_ig(code, ig, &ts))
        << "Bar CSV 喂完后应至少产出一次 IG（确保 [lsig].window_size <= 行数）";
    EXPECT_TRUE(std::isfinite(ig));
}

// ============== 用例 2：Transaction 驱动（合成数据） ==============
TEST(LeastSquaresInfoGain, TransactionsSyntheticFeedProducesFactor) {
    BusGuard _guard;
    LeastSquaresInfoGainFactor::register_topics(4096);

    auto trans = make_transactions("LSIG_TRX", 150);

    const std::string code = "LSIG_TRX";

    LsInfoGainConfig cfg;
    LeastSquaresInfoGainFactor factor({ code }, cfg);

    for (const auto& t : trans) {
        // IFactor::on_tick(const Transaction&) 会自动包装成 CombinedTick
        factor.on_tick(t);
    }

    double ig = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_ig(code, ig, &ts))
        << "Transaction CSV 喂完后应至少产出一次 IG";
    EXPECT_TRUE(std::isfinite(ig));
}

// ============== 用例 3：Quote 驱动（中间价）===============
TEST(LeastSquaresInfoGain, QuotesSyntheticFeedProducesFactor) {
    BusGuard _guard;
    LeastSquaresInfoGainFactor::register_topics(4096);

    auto quotes = make_quotes("LSIG_Q", 140);

    const std::string code = "LSIG_Q";

    LsInfoGainConfig cfg;
    LeastSquaresInfoGainFactor factor({ code }, cfg);

    for (const auto& q : quotes) {
        factor.on_quote(q);
    }

    double ig = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_ig(code, ig, &ts))
        << "Quote CSV 喂完后应至少产出一次 IG";
    EXPECT_TRUE(std::isfinite(ig));
}

// ============== 用例 4：三种数据混合驱动 ==============
TEST(LeastSquaresInfoGain, MixedSyntheticFeedMultiSources) {
    BusGuard _guard;
    LeastSquaresInfoGainFactor::register_topics(8192);

    const std::string code = "LSIG_MIX";
    auto bars   = make_bars(code, 80);
    auto trans  = make_transactions(code, 90);
    auto quotes = make_quotes(code, 85);

    LsInfoGainConfig cfg;
    LeastSquaresInfoGainFactor factor({ code }, cfg);

    // 顺序无所谓：Bar -> Transaction -> Quote，三种数据混合驱动
    for (const auto& b : bars)   factor.on_bar(b);
    for (const auto& t : trans)  factor.on_tick(t);
    for (const auto& q : quotes) factor.on_quote(q);

    double ig = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_ig(code, ig, &ts))
        << "三种 CSV 混合喂完后应至少产出一次 IG";
    EXPECT_TRUE(std::isfinite(ig));
}
