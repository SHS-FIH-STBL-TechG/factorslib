// tests/stat_factors_tests/hawkes_intensity_factor_test.cpp

#include "factors/stat/hawkes_intensity_factor.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "core/databus.h"
#include "core/scope_key.h"
#include "core/types.h"

using namespace factorlib;

// 为了测试，把 HawkesIntensityFactor 包一层，补齐 on_quote，使之非抽象
struct HawkesIntensityFactorForTest : public HawkesIntensityFactor {
    using HawkesIntensityFactor::HawkesIntensityFactor;

    void on_quote(const QuoteDepth& /*q*/) override {
        // 对于本测试，我们只关心 on_tick 的行为，这里留空即可
    }
};

namespace {

struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

bool last_hawkes(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_HAWKES, code, out, ts);
}

Transaction make_trade(const std::string& code, int64_t ts_ms, uint64_t seq = 1,
                       double price = 1.0, uint64_t volume = 1) {
    Transaction tr;
    tr.instrument_id = code;
    tr.data_time_ms  = ts_ms;
    tr.main_seq      = seq;
    tr.price         = price;
    tr.side          = 1;
    tr.volume        = volume;
    tr.bid_no        = 0;
    tr.ask_no        = 0;
    return tr;
}

Entrust make_order(const std::string& code, int64_t ts_ms, uint64_t seq = 1,
                   double price = 1.0, uint64_t volume = 1) {
    Entrust en;
    en.instrument_id = code;
    en.data_time_ms  = ts_ms;
    en.main_seq      = seq;
    en.price         = price;
    en.side          = -1;
    en.volume        = volume;
    en.order_id      = seq;
    return en;
}

// This helper function models the discrete-time update rule that the underlying
// `math::HawkesIntensity` implementation is assumed to use. The decay is based on
// a fixed `cfg.dt` rather than the actual time delta between events.
double discrete_hawkes_step(double prev_lambda, const HawkesCfg& cfg, double events) {
    const double decay = std::exp(-cfg.beta * cfg.dt);
    return cfg.mu + decay * (prev_lambda - cfg.mu) + cfg.alpha * events;
}

std::vector<double> discrete_hawkes_sequence(const HawkesCfg& cfg,
                                             const std::vector<double>& events,
                                             double initial_lambda = std::numeric_limits<double>::quiet_NaN()) {
    std::vector<double> seq;
    seq.reserve(events.size());
    double lambda_prev = std::isnan(initial_lambda) ? cfg.mu : initial_lambda;
    for (double ev : events) {
        lambda_prev = discrete_hawkes_step(lambda_prev, cfg, ev);
        seq.push_back(lambda_prev);
    }
    return seq;
}

constexpr double kTol = 1e-9;

} // namespace

class HawkesIntensityFactorTest : public ::testing::Test {
protected:
    BusGuard guard_;
};

TEST_F(HawkesIntensityFactorTest, SyntheticTransactions_MatchDiscreteModel) {
    HawkesIntensityFactor::register_topics(2048);

    const std::string code = "TEST_HAWKES";

    // 选一组简单参数，便于计算
    HawkesCfg cfg;
    cfg.mu         = 0.1;
    cfg.alpha      = 0.3;
    cfg.beta       = 1.0;
    cfg.dt         = 1.0;    // Δt = 1s

    HawkesIntensityFactorForTest factor({code}, cfg);

    // 构造一段人造成交序列：固定时间间隔 Δt = 1s
    const int N = 20;
    const int64_t base_ms = 1'000'000;   // 任意起始时间
    const int64_t step_ms = static_cast<int64_t>(cfg.dt * 1000.0);

    std::vector<double> lambda_factor;   // 从因子读出的 λ_k
    lambda_factor.reserve(N);

    for (int k = 0; k < N; ++k) {
        Transaction tr;
        tr.instrument_id = code;
        tr.data_time_ms  = base_ms + k * step_ms;
        tr.main_seq      = static_cast<uint64_t>(k + 1);
        tr.price         = 1.0;
        tr.side          = 1;
        tr.volume        = 1;
        tr.bid_no        = 0;
        tr.ask_no        = 0;

        // IFactor 里已有 from Transaction 的 on_tick 重载
        factor.on_tick(tr);

        double lambda = 0.0;
        int64_t ts    = 0;
        ASSERT_TRUE(last_hawkes(code, lambda, &ts))
            << "第 " << k << " 个事件后未能从 DataBus 读到 Hawkes 强度";

        ASSERT_TRUE(std::isfinite(lambda));
        lambda_factor.push_back(lambda);
    }

    // ========= 在测试端按离散 Hawkes 模型重算 λ_k =========
    std::vector<double> events(N, 1.0);
    const auto lambda_expected = discrete_hawkes_sequence(cfg, events, cfg.mu);

    ASSERT_EQ(lambda_expected.size(), lambda_factor.size());

    // ========= 对比两边序列 =========
    for (int k = 0; k < N; ++k) {
        double lf = lambda_factor[k];
        double le = lambda_expected[k];

        double diff = std::fabs(lf - le);
        double tol  = std::max(kTol, 1e-8 * std::max(1.0, std::fabs(le)));

        EXPECT_LE(diff, tol) << "k=" << k
                             << " factor=" << lf
                             << " expected=" << le;
    }
}

TEST_F(HawkesIntensityFactorTest, PublishesScopedCodeAfterTrade) {
    HawkesIntensityFactor::register_topics(32);

    const std::string code = "SCOPED";
    HawkesCfg cfg;
    cfg.mu = 0.2;
    cfg.alpha = 0.5;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t ts = 123456;
    factor.on_tick(make_trade(code, ts, 1));

    const auto scoped_code = compose_scope_code(code, 0);
    double lambda = 0.0;
    ASSERT_TRUE(last_hawkes(scoped_code, lambda));

    const double expected = discrete_hawkes_step(cfg.mu, cfg, 1.0);
    EXPECT_NEAR(lambda, expected, kTol);
}

// This test verifies that the DataBus correctly aliases the base code to the
// latest value published on any of its scoped sub-topics. This is considered
// a feature of the DataBus infrastructure.
TEST_F(HawkesIntensityFactorTest, BaseCodeAliasReturnsLatestValue) {
    HawkesIntensityFactor::register_topics(32);

    const std::string code = "ALIAS";
    HawkesCfg cfg;
    cfg.mu = 0.15;
    cfg.alpha = 0.25;
    HawkesIntensityFactorForTest factor({code}, cfg);

    factor.on_tick(make_trade(code, 77'000, 1));

    double scoped_value = 0.0;
    double base_value = 0.0;
    ASSERT_TRUE(last_hawkes(compose_scope_code(code, 0), scoped_value));
    ASSERT_TRUE(last_hawkes(code, base_value));

    EXPECT_NEAR(scoped_value, base_value, kTol);
}

TEST_F(HawkesIntensityFactorTest, TimestampMatchesEventTime) {
    HawkesIntensityFactor::register_topics(32);

    const std::string code = "TS_CHECK";
    HawkesCfg cfg;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t ts = 9'999'123;
    factor.on_tick(make_trade(code, ts, 1));

    double value = 0.0;
    int64_t recorded_ts = 0;
    ASSERT_TRUE(last_hawkes(code, value, &recorded_ts));
    EXPECT_EQ(recorded_ts, ts);
}

TEST_F(HawkesIntensityFactorTest, OrderEventTriggersDecayPublish) {
    HawkesIntensityFactor::register_topics(32);

    const std::string code = "ORD_DECAY";
    HawkesCfg cfg;
    cfg.mu = 0.12;
    cfg.alpha = 0.4;
    cfg.beta = 0.8;
    cfg.dt = 1.0;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t base_ts = 50'000;
    const int64_t step_ms = static_cast<int64_t>(cfg.dt * 1000);

    factor.on_tick(make_trade(code, base_ts, 1));
    double after_trade = 0.0;
    ASSERT_TRUE(last_hawkes(code, after_trade));

    factor.on_tick(make_order(code, base_ts + step_ms, 2));
    double after_order = 0.0;
    ASSERT_TRUE(last_hawkes(code, after_order));

    EXPECT_GT(after_trade, cfg.mu);
    EXPECT_LT(after_order, after_trade);
    EXPECT_GT(after_order, cfg.mu);
}

TEST_F(HawkesIntensityFactorTest, InitialStateIsMu) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "INIT_STATE";
    HawkesCfg cfg;
    cfg.mu = 0.55;
    HawkesIntensityFactorForTest factor({code}, cfg);

    // Before any event, no value should be published.
    double lambda = 0.0;
    EXPECT_FALSE(last_hawkes(code, lambda));

    // After a first event (even a non-trade one), the state should be initialized
    // to mu, as there is no previous lambda to decay from.
    factor.on_tick(make_order(code, 1'000, 1));
    ASSERT_TRUE(last_hawkes(code, lambda));
    EXPECT_NEAR(lambda, cfg.mu, kTol);
}

TEST_F(HawkesIntensityFactorTest, MultipleCodesIndependentStates) {
    HawkesIntensityFactor::register_topics(64);

    const std::string a = "AAA";
    const std::string b = "BBB";
    HawkesCfg cfg;
    cfg.mu = 0.1;
    cfg.alpha = 0.3;
    HawkesIntensityFactorForTest factor({a, b}, cfg);

    factor.on_tick(make_trade(a, 1'000, 1));
    double val_a = 0.0;
    ASSERT_TRUE(last_hawkes(a, val_a));
    double dummy = 0.0;
    EXPECT_FALSE(last_hawkes(b, dummy));

    factor.on_tick(make_trade(b, 2'000, 1));
    double val_b = 0.0;
    ASSERT_TRUE(last_hawkes(b, val_b));
    EXPECT_NEAR(val_b, discrete_hawkes_step(cfg.mu, cfg, 1.0), kTol);
    double val_a_again = 0.0;
    ASSERT_TRUE(last_hawkes(a, val_a_again));
    EXPECT_DOUBLE_EQ(val_a_again, val_a);
}

TEST_F(HawkesIntensityFactorTest, SequentialTradesAccumulateActivation) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "SEQUENTIAL";
    HawkesCfg cfg;
    cfg.mu = 0.1;
    cfg.alpha = 0.2;
    cfg.beta = 0.7;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t base_ts = 9'000;
    const int64_t step = static_cast<int64_t>(cfg.dt * 1000);

    factor.on_tick(make_trade(code, base_ts, 1));
    double first = 0.0;
    ASSERT_TRUE(last_hawkes(code, first));

    factor.on_tick(make_trade(code, base_ts + step, 2));
    double second = 0.0;
    ASSERT_TRUE(last_hawkes(code, second));

    EXPECT_GT(second, first);

    const double expected_first = discrete_hawkes_step(cfg.mu, cfg, 1.0);
    const double expected_second = discrete_hawkes_step(expected_first, cfg, 1.0);
    EXPECT_NEAR(first, expected_first, kTol);
    EXPECT_NEAR(second, expected_second, kTol);
}

TEST_F(HawkesIntensityFactorTest, EntrustConversionUpdatesOrderPath) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "ENTRUST";
    HawkesCfg cfg;
    cfg.mu = 0.2;
    cfg.alpha = 0.4;
    cfg.beta = 0.9;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t base_ts = 100'000;
    const int64_t step = static_cast<int64_t>(cfg.dt * 1000);

    factor.on_tick(make_trade(code, base_ts, 1));
    double after_trade = 0.0;
    ASSERT_TRUE(last_hawkes(code, after_trade));

    factor.on_tick(make_order(code, base_ts + step, 2));
    double after_order = 0.0;
    ASSERT_TRUE(last_hawkes(code, after_order));

    const double expected_after_order = discrete_hawkes_step(after_trade, cfg, 0.0);
    EXPECT_NEAR(after_order, expected_after_order, kTol);
}

TEST_F(HawkesIntensityFactorTest, CustomDtAffectsDecay) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "CUSTOM_DT";
    HawkesCfg cfg;
    cfg.mu = 0.05;
    cfg.alpha = 0.3;
    cfg.beta = 0.4;
    cfg.dt = 2.0;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t base_ts = 500;
    const int64_t step = static_cast<int64_t>(cfg.dt * 1000);

    factor.on_tick(make_trade(code, base_ts, 1));
    double first = 0.0;
    ASSERT_TRUE(last_hawkes(code, first));

    factor.on_tick(make_trade(code, base_ts + step, 2));
    double second = 0.0;
    ASSERT_TRUE(last_hawkes(code, second));

    const double expected_first = discrete_hawkes_step(cfg.mu, cfg, 1.0);
    const double expected_second = discrete_hawkes_step(expected_first, cfg, 1.0);
    EXPECT_NEAR(first, expected_first, kTol);
    EXPECT_NEAR(second, expected_second, kTol);
}

TEST_F(HawkesIntensityFactorTest, ZeroAlphaPreventsExcitation) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "ZERO_ALPHA";
    HawkesCfg cfg;
    cfg.mu = 0.18;
    cfg.alpha = 0.0;
    cfg.beta = 0.7;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t base_ts = 1'000;
    const int64_t step = static_cast<int64_t>(cfg.dt * 1000);

    for (int i = 0; i < 3; ++i) {
        factor.on_tick(make_trade(code, base_ts + i * step, i + 1));
        double lambda = 0.0;
        ASSERT_TRUE(last_hawkes(code, lambda));
        EXPECT_NEAR(lambda, cfg.mu, kTol);
    }
}

TEST_F(HawkesIntensityFactorTest, CustomMuPropagatesThroughOrder) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "CUSTOM_MU";
    HawkesCfg cfg;
    cfg.mu = 0.42;
    cfg.alpha = 0.25;
    HawkesIntensityFactorForTest factor({code}, cfg);

    factor.on_tick(make_order(code, 222'000, 1));
    double lambda = 0.0;
    ASSERT_TRUE(last_hawkes(code, lambda));
    EXPECT_NEAR(lambda, cfg.mu, kTol);
}

TEST_F(HawkesIntensityFactorTest, ForceFlushIsNoOp) {
    HawkesIntensityFactor::register_topics(32);
    HawkesCfg cfg;
    HawkesIntensityFactorForTest factor({"CODE"}, cfg);
    EXPECT_FALSE(factor.force_flush("CODE"));
    EXPECT_FALSE(factor.force_flush("UNKNOWN"));
}

TEST_F(HawkesIntensityFactorTest, HistoryStoresAllEvents) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "HISTORY";
    HawkesCfg cfg;
    cfg.mu = 0.1;
    cfg.alpha = 0.3;
    cfg.beta = 0.4;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t base_ts = 10'000;
    const int64_t step = static_cast<int64_t>(cfg.dt * 1000);

    factor.on_tick(make_trade(code, base_ts, 1));
    factor.on_tick(make_order(code, base_ts + step, 2));
    factor.on_tick(make_trade(code, base_ts + 2 * step, 3));

    auto history = DataBus::instance().get_last_n<double>(TOP_HAWKES, code, 5);
    ASSERT_EQ(history.size(), 3u);
    EXPECT_EQ(history[0].first, base_ts);
    EXPECT_EQ(history[1].first, base_ts + step);
    EXPECT_EQ(history[2].first, base_ts + 2 * step);
}

TEST_F(HawkesIntensityFactorTest, IgnoresNonMonotonicTimestamps) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "NON_MONO";
    HawkesCfg cfg;
    HawkesIntensityFactorForTest factor({code}, cfg);

    factor.on_tick(make_trade(code, 10'000, 1));
    double first_lambda = 0.0;
    int64_t first_ts = 0;
    ASSERT_TRUE(last_hawkes(code, first_lambda, &first_ts));
    EXPECT_EQ(first_ts, 10'000);

    // Send an event with an older timestamp. The factor's underlying state machine
    // should ignore it to prevent time from going backward.
    factor.on_tick(make_order(code, 9'000, 2));

    double second_lambda = 0.0;
    int64_t second_ts = 0;
    ASSERT_TRUE(last_hawkes(code, second_lambda, &second_ts));

    // The value and timestamp should not have changed from the first event.
    EXPECT_EQ(second_ts, first_ts);
    EXPECT_NEAR(second_lambda, first_lambda, kTol);
}

TEST_F(HawkesIntensityFactorTest, DecaysToMuWithNoEvents) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "DECAY_TO_MU";
    HawkesCfg cfg;
    cfg.mu = 0.1;
    cfg.alpha = 0.8; // High alpha to get a big jump
    cfg.beta = 1.0;
    cfg.dt = 0.5;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t base_ts = 100'000;
    const int64_t step = static_cast<int64_t>(cfg.dt * 1000.0);

    // 1. A trade event to jump the intensity up
    factor.on_tick(make_trade(code, base_ts, 1));
    double lambda = 0.0;
    ASSERT_TRUE(last_hawkes(code, lambda));
    ASSERT_GT(lambda, cfg.mu);

    // 2. Simulate many steps of inactivity by sending non-trade (event=0) events
    double prev_lambda = lambda;
    for (int i = 1; i <= 30; ++i) {
        factor.on_tick(make_order(code, base_ts + i * step, i + 1));
        ASSERT_TRUE(last_hawkes(code, lambda));
        // Should be decaying towards mu
        EXPECT_LT(std::fabs(lambda - cfg.mu), std::fabs(prev_lambda - cfg.mu));
        prev_lambda = lambda;
    }

    // 3. After many steps, it should be very close to mu
    EXPECT_NEAR(lambda, cfg.mu, 1e-5);
}

TEST_F(HawkesIntensityFactorTest, HandlesUnstableCondition) {
    HawkesIntensityFactor::register_topics(32);
    const std::string code = "UNSTABLE";
    HawkesCfg cfg;
    cfg.mu = 0.1;
    cfg.alpha = 1.5; // alpha > beta implies an unstable process
    cfg.beta = 1.0;
    cfg.dt = 1.0;
    HawkesIntensityFactorForTest factor({code}, cfg);

    const int64_t base_ts = 10'000;
    const int64_t step = static_cast<int64_t>(cfg.dt * 1000.0);

    factor.on_tick(make_trade(code, base_ts, 1));
    double prev_lambda = 0.0;
    ASSERT_TRUE(last_hawkes(code, prev_lambda));

    // With each event, the intensity should continue to grow
    for (int i = 1; i < 5; ++i) {
        double lambda = 0.0;
        factor.on_tick(make_trade(code, base_ts + i * step, i + 1));
        ASSERT_TRUE(last_hawkes(code, lambda));
        EXPECT_GT(lambda, prev_lambda);
        prev_lambda = lambda;
    }
}
