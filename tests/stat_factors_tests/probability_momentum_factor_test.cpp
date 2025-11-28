#include "factors/stat/probability_momentum_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <limits>
#include <string>
#include <algorithm>

#include "core/databus.h"
#include "config/runtime_config.h"
#include "math/distributions.h"

using namespace factorlib;

namespace {

class ProbabilityMomentumFixture : public ::testing::Test {
protected:
    void SetUp() override {
        DataBus::instance().reset();
        ProbabilityMomentumFactor::register_topics(2048);
        codes_ = {"PM_TEST"};
        factor_ = std::make_unique<ProbabilityMomentumFactor>(codes_);
        auto& cfg = config::RC();
        window_size_      = cfg.geti("prob_mom.window_size", 60);
        min_total_weight_ = cfg.getd("prob_mom.min_total_weight", 10.0);
        min_sigma_        = cfg.getd("prob_mom.min_sigma", 1e-6);
        buy_threshold_    = cfg.getd("prob_mom.buy_threshold", 0.7);
        sell_threshold_   = cfg.getd("prob_mom.sell_threshold", 0.3);
    }

    void TearDown() override {
        factor_.reset();
        DataBus::instance().reset();
    }

    void reset_factor(const ProbMomentumConfig& cfg = {}) {
        factor_.reset();
        factor_ = std::make_unique<ProbabilityMomentumFactor>(codes_, cfg);
        last_price_anchor_ = 100.0;
        last_ts_ = 0;
    }

    std::vector<double> build_prices(const std::vector<double>& returns, double start_price = 100.0) const {
        std::vector<double> prices;
        prices.reserve(returns.size() + 1);
        prices.push_back(start_price);
        for (double r : returns) {
            prices.push_back(prices.back() * std::exp(r));
        }
        return prices;
    }

    void feed_ticks(const std::vector<double>& returns,
                    const std::vector<double>& volumes,
                    int64_t start_ts,
                    int64_t step_ms,
                    double start_price = 100.0) {
        auto prices = build_prices(returns, start_price);
        CombinedTick tick;
        tick.instrument_id = codes_.front();
        tick.kind = CombinedKind::Trade;
        if (!prices.empty()) {
            tick.data_time_ms = start_ts;
            tick.price = prices.front();
            tick.volume = volumes.empty() ? 1 : static_cast<uint64_t>(std::max<double>(volumes.front(), 1.0));
            factor_->on_tick(tick);
        }
        for (size_t i = 0; i < returns.size(); ++i) {
            tick.data_time_ms = start_ts + static_cast<int64_t>(i + 1) * step_ms;
            tick.price  = prices[i + 1];
            tick.volume = static_cast<uint64_t>(std::max<double>(volumes[i], 0.0));
            factor_->on_tick(tick);
        }
        last_price_anchor_ = prices.back();
        last_ts_ = start_ts + static_cast<int64_t>(returns.size()) * step_ms;
    }

    void feed_quotes(const std::vector<double>& returns,
                     const std::vector<double>& volumes,
                     int64_t start_ts,
                     int64_t step_ms,
                     double start_price = 100.0) {
        auto prices = build_prices(returns, start_price);
        QuoteDepth q;
        q.instrument_id = codes_.front();
        if (!prices.empty()) {
            q.data_time_ms = start_ts;
            q.bid_price = prices.front() - 0.01;
            q.ask_price = prices.front() + 0.01;
            q.volume = volumes.empty() ? 1 : static_cast<uint64_t>(std::max<double>(volumes.front(), 1.0));
            factor_->on_quote(q);
        }
        for (size_t i = 0; i < returns.size(); ++i) {
            q.data_time_ms = start_ts + static_cast<int64_t>(i + 1) * step_ms;
            q.bid_price = prices[i + 1] - 0.01;
            q.ask_price = prices[i + 1] + 0.01;
            q.volume = static_cast<uint64_t>(std::max<double>(volumes[i], 0.0));
            factor_->on_quote(q);
        }
        last_price_anchor_ = prices.back();
        last_ts_ = start_ts + static_cast<int64_t>(returns.size()) * step_ms;
    }

    void feed_bars(const std::vector<double>& returns,
                   const std::vector<double>& volumes,
                   int64_t start_ts,
                   int64_t step_ms,
                   double start_price = 100.0) {
        auto prices = build_prices(returns, start_price);
        Bar b;
        b.instrument_id = codes_.front();
        if (!prices.empty()) {
            b.data_time_ms = start_ts;
            b.open = prices.front();
            b.close = prices.front();
            b.high = prices.front();
            b.low = prices.front();
            b.volume = volumes.empty() ? 1 : static_cast<uint64_t>(std::max<double>(volumes.front(), 1.0));
            b.turnover = b.close * b.volume;
            b.interval_ms = static_cast<int32_t>(step_ms);
            factor_->on_bar(b);
        }
        for (size_t i = 0; i < returns.size(); ++i) {
            b.data_time_ms = start_ts + static_cast<int64_t>(i + 1) * step_ms;
            b.open = prices[i];
            b.close = prices[i + 1];
            b.high = std::max(prices[i], prices[i + 1]);
            b.low = std::min(prices[i], prices[i + 1]);
            b.volume = static_cast<uint64_t>(std::max<double>(volumes[i], 0.0));
            b.turnover = b.close * b.volume;
            b.interval_ms = static_cast<int32_t>(step_ms);
            factor_->on_bar(b);
        }
        last_price_anchor_ = prices.back();
        last_ts_ = start_ts + static_cast<int64_t>(returns.size()) * step_ms;
    }

    std::vector<double> returns_with_value(double value) const {
        return std::vector<double>(window_size_, value);
    }

    std::string scoped_code() const {
        return compose_scope_code(codes_.front(), window_size_);
    }

    bool latest_prob(double& out, int64_t* ts = nullptr) const {
        return DataBus::instance().get_latest<double>(TOP_PROB_MOM_PROB, scoped_code(), out, ts);
    }

    bool latest_signal(double& out) const {
        return DataBus::instance().get_latest<double>(TOP_PROB_MOM_SIGNAL, scoped_code(), out, nullptr);
    }

    double compute_expected_prob(const std::vector<double>& returns,
                                 const std::vector<double>& volumes) const {
        if (returns.empty()) return std::numeric_limits<double>::quiet_NaN();
        size_t usable = std::min(returns.size(), static_cast<size_t>(window_size_));
        size_t start = returns.size() - usable;
        long double sum_w = 0.0L;
        long double sum_wr = 0.0L;
        long double sum_wr2 = 0.0L;
        for (size_t i = start; i < returns.size(); ++i) {
            long double w = std::max<long double>(volumes[i], 0.0L);
            if (!(w > 0.0L)) w = 1.0L;
            long double r = returns[i];
            sum_w   += w;
            sum_wr  += w * r;
            sum_wr2 += w * r * r;
        }
        if (sum_w < min_total_weight_) return std::numeric_limits<double>::quiet_NaN();
        long double mu = sum_wr / sum_w;
        long double var = sum_wr2 / sum_w - mu * mu;
        if (var < 0.0L) var = 0.0L;
        double sigma = std::sqrt(static_cast<double>(var));
        if (!(sigma > 0.0)) sigma = min_sigma_;
        double z = static_cast<double>(mu) / sigma;
        return math::Distributions<>::normal_cdf(z);
    }

    std::unique_ptr<ProbabilityMomentumFactor> factor_;
    std::vector<std::string> codes_;
    int window_size_{60};
    double min_total_weight_{10.0};
    double min_sigma_{1e-6};
    double buy_threshold_{0.7};
    double sell_threshold_{0.3};
    double last_price_anchor_{100.0};
    int64_t last_ts_{0};
};

// 用例：连续上涨行情，价格每期上涨 0.2%，应使概率高于买入阈值并输出 +1 信号。
TEST_F(ProbabilityMomentumFixture, QuotesUpTrendProducesBuySignal) {
    auto returns = returns_with_value(0.002);              // 每期上涨 0.2%
    std::vector<double> vols(returns.size(), 200.0);       // 固定权重
    feed_quotes(returns, vols, 0, 5);

    double prob = 0.0;
    ASSERT_TRUE(latest_prob(prob));
    EXPECT_GT(prob, buy_threshold_);

    double signal = 0.0;
    ASSERT_TRUE(latest_signal(signal));
    EXPECT_DOUBLE_EQ(signal, 1.0);
}

// 用例：连续下跌行情，每期 -0.25%，概率应低于卖出阈值并输出 -1 信号。
TEST_F(ProbabilityMomentumFixture, QuotesDownTrendProducesSellSignal) {
    auto returns = returns_with_value(-0.0025);            // 每期下跌 0.25%
    std::vector<double> vols(returns.size(), 300.0);
    feed_quotes(returns, vols, 0, 5);
    double prob = 0.0;
    ASSERT_TRUE(latest_prob(prob));
    EXPECT_LT(prob, sell_threshold_);
    double signal = 0.0;
    ASSERT_TRUE(latest_signal(signal));
    EXPECT_DOUBLE_EQ(signal, -1.0);
}

// 用例：涨跌交替且幅度对称，概率应接近 0.5，信号保持 0。
TEST_F(ProbabilityMomentumFixture, AlternatingTrendKeepsSignalNeutral) {
    std::vector<double> returns;
    std::vector<double> vols;
    for (int i = 0; i < window_size_; ++i) {
        returns.push_back((i % 2 == 0) ? 0.001 : -0.001);
        vols.push_back(150.0);
    }
    feed_quotes(returns, vols, 0, 4);
    double prob = 0.0;
    ASSERT_TRUE(latest_prob(prob));
    EXPECT_NEAR(prob, 0.5, 5e-2);
    double signal = 0.0;
    ASSERT_TRUE(latest_signal(signal));
    EXPECT_DOUBLE_EQ(signal, 0.0);
}

// 用例：后期出现大成交量的负收益，权重应让概率迅速跌破卖出阈值。
TEST_F(ProbabilityMomentumFixture, TickWeightsDominatedByLargeSellVolume) {
    auto returns = returns_with_value(0.001);
    std::vector<double> vols(returns.size(), 50.0);
    for (size_t i = vols.size() - 10; i < vols.size(); ++i) {
        returns[i] = -0.01;
        vols[i] = 1000.0;
    }
    feed_ticks(returns, vols, 0, 3);
    double prob = 0.0;
    ASSERT_TRUE(latest_prob(prob));
    EXPECT_LT(prob, sell_threshold_);
}

// 用例：Bar 喂数与 Tick 喂数应得到完全一致的结果（数值与时间戳）。
TEST_F(ProbabilityMomentumFixture, BarInputMatchesTickResult) {
    auto returns = returns_with_value(0.0015);
    std::vector<double> vols(returns.size(), 180.0);
    feed_bars(returns, vols, 1000, 8);

    double bar_prob = 0.0;
    int64_t bar_ts = 0;
    ASSERT_TRUE(latest_prob(bar_prob, &bar_ts));

    DataBus::instance().reset();
    ProbabilityMomentumFactor::register_topics(2048);
    reset_factor();
    feed_ticks(returns, vols, 1000, 8);

    double tick_prob = 0.0;
    int64_t tick_ts = 0;
    ASSERT_TRUE(latest_prob(tick_prob, &tick_ts));
    EXPECT_NEAR(bar_prob, tick_prob, 1e-9);
    EXPECT_EQ(bar_ts, tick_ts);
}

// 用例：在未填满窗口（N-1 条）之前不应发布结果，填满窗口后立即发布。
TEST_F(ProbabilityMomentumFixture, LessThanWindowProducesNoOutput) {
    std::vector<double> returns(window_size_ - 1, 0.002);
    std::vector<double> vols(returns.size(), 200.0);
    feed_ticks(returns, vols, 0, 5);
    double prob = 0.0;
    EXPECT_FALSE(latest_prob(prob));

    std::vector<double> extra(1, 0.002);
    std::vector<double> extra_vol(1, 200.0);
    feed_ticks(extra, extra_vol, last_ts_ + 5, 5, last_price_anchor_);
    ASSERT_TRUE(latest_prob(prob));
}

// 用例：人为设置极大的 min_total_weight，使得权重不足时不产出结果。
TEST_F(ProbabilityMomentumFixture, HugeMinWeightBlocksOutput) {
    ProbMomentumConfig cfg;
    cfg.window_size = window_size_;
    cfg.min_total_weight = 1e9;
    cfg.min_sigma = min_sigma_;
    cfg.buy_threshold = buy_threshold_;
    cfg.sell_threshold = sell_threshold_;
    DataBus::instance().reset();
    ProbabilityMomentumFactor::register_topics(2048);
    reset_factor(cfg);

    auto returns = returns_with_value(0.002);
    std::vector<double> vols(returns.size(), 100.0);
    feed_ticks(returns, vols, 0, 5);
    double prob = 0.0;
    EXPECT_FALSE(latest_prob(prob));
}

// 用例：收益率全为 0，方差为 0，概率应退化到 0.5。
TEST_F(ProbabilityMomentumFixture, ZeroVarianceFallsBackToHalfProbability) {
    auto returns = returns_with_value(0.0);
    std::vector<double> vols(returns.size(), 300.0);
    feed_ticks(returns, vols, 0, 5);
    double prob = 0.0;
    ASSERT_TRUE(latest_prob(prob));
    EXPECT_NEAR(prob, 0.5, 1e-9);
}

// 用例：无效报价（bid<=0）应被忽略，概率维持不变。
TEST_F(ProbabilityMomentumFixture, InvalidQuoteIgnored) {
    auto returns = returns_with_value(0.002);
    std::vector<double> vols(returns.size(), 200.0);
    feed_quotes(returns, vols, 0, 5);
    double baseline = 0.0;
    ASSERT_TRUE(latest_prob(baseline));

    QuoteDepth invalid{};
    invalid.instrument_id = codes_.front();
    invalid.data_time_ms = last_ts_ + 1;
    invalid.bid_price = 0.0;
    invalid.ask_price = 10.0;
    factor_->on_quote(invalid);

    double after = 0.0;
    ASSERT_TRUE(latest_prob(after));
    EXPECT_DOUBLE_EQ(after, baseline);
}

// 用例：volume=0 时权重应自动回退到 1，结果与明确定义权重 1 一致。
TEST_F(ProbabilityMomentumFixture, ZeroVolumeTreatedAsUnitWeight) {
    auto returns = returns_with_value(0.001);
    std::vector<double> vols(returns.size(), 0.0);
    feed_ticks(returns, vols, 0, 4);
    double prob = 0.0;
    ASSERT_TRUE(latest_prob(prob));
    auto expected = compute_expected_prob(returns, std::vector<double>(returns.size(), 1.0));
    EXPECT_NEAR(prob, expected, 1e-9);
}

// 用例：volume 为 NaN 的样本被 BadValuePolicy 丢弃，其余样本仍能给出正确结果。
TEST_F(ProbabilityMomentumFixture, NaNVolumeIsDroppedByBadValuePolicy) {
    auto returns = returns_with_value(0.001);
    std::vector<double> vols(returns.size(), 100.0);
    vols.back() = std::numeric_limits<double>::quiet_NaN();
    feed_ticks(returns, vols, 0, 4);

    double prob = 0.0;
    ASSERT_TRUE(latest_prob(prob));
    auto expected = compute_expected_prob(returns, vols);
    double fallback = compute_expected_prob(returns, std::vector<double>(returns.size() - 1, 100.0));
    if (std::isnan(expected)) {
        EXPECT_NEAR(prob, fallback, 1e-6);
    } else {
        EXPECT_NEAR(prob, expected, 1e-6);
    }
}

// 用例：遇到 NaN 价格后重置链条，再次喂入负收益应将概率压低。
TEST_F(ProbabilityMomentumFixture, NonFiniteTickPriceResetsChain) {
    auto returns = returns_with_value(0.002);
    std::vector<double> vols(returns.size(), 200.0);
    feed_ticks(returns, vols, 0, 5);
    double baseline = 0.0;
    ASSERT_TRUE(latest_prob(baseline));

    CombinedTick bad{};
    bad.instrument_id = codes_.front();
    bad.data_time_ms = last_ts_ + 1;
    bad.price = std::numeric_limits<double>::quiet_NaN();
    bad.volume = 500;
    factor_->on_tick(bad);

    auto extra = returns_with_value(-0.0015);
    std::vector<double> extra_vols(extra.size(), 300.0);
    feed_ticks(extra, extra_vols, last_ts_ + 10, 5, last_price_anchor_);

    double prob = 0.0;
    ASSERT_TRUE(latest_prob(prob));
    EXPECT_LT(prob, 0.5);
}

// 用例：先喂负收益再喂正收益，后者概率应明显高于前者。
TEST_F(ProbabilityMomentumFixture, ProbabilityIncreasesAfterStrongerReturns) {
    auto weak = returns_with_value(-0.001);
    std::vector<double> vols(weak.size(), 150.0);
    feed_ticks(weak, vols, 0, 5);
    double p1 = 0.0;
    ASSERT_TRUE(latest_prob(p1));

    auto strong = returns_with_value(0.002);
    feed_ticks(strong, vols, last_ts_ + 10, 5, last_price_anchor_);
    double p2 = 0.0;
    ASSERT_TRUE(latest_prob(p2));
    EXPECT_GT(p2, p1);
}

// 用例：DataBus 中的时间戳应与最后一条事件时间完全一致。
TEST_F(ProbabilityMomentumFixture, LatestTimestampMatchesLastTick) {
    auto returns = returns_with_value(0.001);
    std::vector<double> vols(returns.size(), 220.0);
    feed_ticks(returns, vols, 5000, 7);
    double prob = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(latest_prob(prob, &ts));
    EXPECT_EQ(ts, last_ts_);
}

// 用例：因子未实现强制刷新，即便窗口满也应返回 false。
TEST_F(ProbabilityMomentumFixture, ForceFlushAlwaysReturnsFalse) {
    auto returns = returns_with_value(0.002);
    std::vector<double> vols(returns.size(), 250.0);
    feed_ticks(returns, vols, 0, 5);
    EXPECT_FALSE(factor_->force_flush(codes_.front()));
}

// 用例：同样的价格路径，无论 Tick 还是 Quote 喂入概率都应一致。
TEST_F(ProbabilityMomentumFixture, TickAndQuoteProduceIdenticalProbabilities) {
    auto returns = returns_with_value(0.0012);
    std::vector<double> vols(returns.size(), 140.0);
    feed_ticks(returns, vols, 0, 5);
    double tick_prob = 0.0;
    ASSERT_TRUE(latest_prob(tick_prob));

    DataBus::instance().reset();
    ProbabilityMomentumFactor::register_topics(2048);
    reset_factor();
    feed_quotes(returns, vols, 0, 5);
    double quote_prob = 0.0;
    ASSERT_TRUE(latest_prob(quote_prob));
    EXPECT_NEAR(tick_prob, quote_prob, 1e-9);
}

// 用例：前几条 NaN 价格被忽略，后续正常数据仍应给出结果。
TEST_F(ProbabilityMomentumFixture, LeadingNanPricesAreIgnored) {
    // 前几条 NaN 价格被忽略后，后续正常数据仍能产出概率
    CombinedTick bad{};
    bad.instrument_id = codes_.front();
    bad.kind = CombinedKind::Trade;
    bad.volume = 100;
    for (int i = 0; i < 3; ++i) {
        bad.price = std::numeric_limits<double>::quiet_NaN();
        bad.data_time_ms = i * 5;
        factor_->on_tick(bad);
    }

    auto returns = returns_with_value(0.0015);
    std::vector<double> vols(returns.size(), 180.0);
    feed_ticks(returns, vols, 100, 5);

    double prob = 0.0;
    ASSERT_TRUE(latest_prob(prob));
    EXPECT_GT(prob, buy_threshold_ - 0.05);
}

// 用例：先喂正收益触发买入信号，再喂负收益触发卖出信号，验证阈值切换。
TEST_F(ProbabilityMomentumFixture, LatestSignalTracksProbabilityThresholds) {
    // 先喂正收益 -> 信号 +1，再喂负收益 -> 信号 -1
    auto up = returns_with_value(0.002);
    std::vector<double> vols(up.size(), 200.0);
    feed_ticks(up, vols, 0, 5);
    double signal = 0.0;
    ASSERT_TRUE(latest_signal(signal));
    EXPECT_DOUBLE_EQ(signal, 1.0);

    auto down = returns_with_value(-0.002);
    feed_ticks(down, vols, last_ts_ + 10, 5, last_price_anchor_);
    ASSERT_TRUE(latest_signal(signal));
    EXPECT_DOUBLE_EQ(signal, -1.0);
}

} // namespace
