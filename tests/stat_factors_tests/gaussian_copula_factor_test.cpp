// tests/stat_factors_tests/gaussian_copula_factor_test.cpp

#include "factors/stat/gaussian_copula_factor.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "core/databus.h"
#include "core/scope_key.h"
#include "core/types.h"
#include "math/distributions.h"
#include "math/linear_algebra.h"

using namespace factorlib;

namespace {

constexpr int64_t kStartTs = 1'000'000;
constexpr int64_t kQuoteStep = 10;
constexpr const char* kCopulaTopic = "gaussian_copula/prediction";

struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

struct Sample {
    int ofi;
    int volume;
    double log_return;
};

struct ReferenceCopulaModel {
    explicit ReferenceCopulaModel(size_t window, double reg)
        : window_size_(window),
          regularization_(reg),
          cov_calc_(window) {}

    void push(double ofi, double volume, double ret) {
        ofi_rank_.push(ofi, window_size_);
        volume_rank_.push(volume, window_size_);
        return_rank_.push(ret, window_size_);

        if (ofi_rank_.size() >= window_size_ &&
            volume_rank_.size() >= window_size_ &&
            return_rank_.size() >= window_size_) {
            const double ofi_rank = ofi_rank_.median_rank(ofi);
            const double vol_rank = volume_rank_.median_rank(volume);
            const double ret_rank = return_rank_.median_rank(ret);

            Eigen::Vector3d normal;
            normal << math::Distributions<double>::normal_quantile(ofi_rank),
                       math::Distributions<double>::normal_quantile(vol_rank),
                       math::Distributions<double>::normal_quantile(ret_rank);
            cov_calc_.push(normal);
        }
    }

    bool is_window_full() const {
        return ofi_rank_.size() >= window_size_ &&
               volume_rank_.size() >= window_size_ &&
               return_rank_.size() >= window_size_ &&
               cov_calc_.size() >= window_size_;
    }

    double predict(double current_ofi, double current_volume) const {
        auto mean = cov_calc_.mean();
        auto covariance = cov_calc_.covariance();
        covariance += Eigen::Matrix3d::Identity() * regularization_;

        const double ofi_rank = ofi_rank_.median_rank(current_ofi);
        const double vol_rank = volume_rank_.median_rank(current_volume);
        Eigen::Vector2d condition;
        condition << math::Distributions<double>::normal_quantile(ofi_rank),
                     math::Distributions<double>::normal_quantile(vol_rank);

        const double conditional_mean =
            math::LinearAlgebra<double>::conditional_expectation(mean, covariance, condition, 2);
        const double prob = 0.5 * (1.0 + std::erf(conditional_mean / std::sqrt(2.0)));
        auto sorted_returns = return_rank_.get_sorted_data();
        return math::Distributions<double>::empirical_inverse_cdf(sorted_returns, prob);
    }

    size_t window_size() const { return window_size_; }

private:
    size_t window_size_;
    double regularization_;
    math::IncrementalRankCalculator<double> ofi_rank_;
    math::IncrementalRankCalculator<double> volume_rank_;
    math::IncrementalRankCalculator<double> return_rank_;
    math::IncrementalCovariance<double, 3> cov_calc_;
};

Entrust make_order(const std::string& code, int64_t ts, int side, uint64_t volume) {
    Entrust e{};
    e.instrument_id = code;
    e.data_time_ms  = ts;
    e.price         = 100.0;
    e.side          = side;
    e.volume        = volume;
    e.order_id      = ts;
    return e;
}

QuoteDepth make_quote(const std::string& code, int64_t ts, double mid) {
    QuoteDepth q{};
    q.instrument_id = code;
    q.data_time_ms  = ts;
    q.bid_price = mid - 0.01;
    q.ask_price = mid + 0.01;
    return q;
}

void feed_orders(GaussianCopulaFactor& factor,
                 const std::string& code,
                 int64_t ts,
                 int ofi,
                 int volume,
                 bool split) {
    ASSERT_GE(volume, std::abs(ofi));
    ASSERT_EQ((volume + ofi) & 1, 0) << "volume 与 ofi 需满足可拆分条件";

    const int buy = (volume + ofi) / 2;
    const int sell = volume - buy;

    auto emit = [&](int amount, int side, int64_t t) {
        if (amount <= 0) return;
        factor.on_tick(make_order(code, t, side, static_cast<uint64_t>(amount)));
    };

    if (split && buy > 1) {
        int first = buy / 2;
        emit(first, +1, ts);
        emit(buy - first, +1, ts + 1);
    } else {
        emit(buy, +1, ts);
    }

    if (split && sell > 1) {
        int first = sell / 2;
        emit(first, -1, ts + 2);
        emit(sell - first, -1, ts + 3);
    } else {
        emit(sell, -1, ts + 2);
    }
}

std::vector<Sample> build_sample_sequence(int length) {
    std::vector<Sample> seq;
    seq.reserve(length);
    for (int i = 0; i < length; ++i) {
        int base_vol = 60 + (i % 5) * 6;
        int ofi = ((i % 7) - 3) * 4;
        if (std::abs(ofi) > base_vol) {
            base_vol = std::abs(ofi) + 4;
        }
        if (((base_vol + ofi) & 1) != 0) ++base_vol;
        double log_ret = 0.001 * (static_cast<double>(i % 9) - 4.0);
        seq.push_back({ofi, base_vol, log_ret});
    }
    return seq;
}

std::vector<double> read_predictions(const std::string& scoped_code) {
    auto rows = DataBus::instance().get_last_n<double>(kCopulaTopic, scoped_code, 2048);
    std::vector<double> values;
    values.reserve(rows.size());
    for (const auto& row : rows) values.push_back(row.second);
    return values;
}

void ensure_volume_alignment(Sample& s) {
    int abs_ofi = std::abs(s.ofi);
    if (s.volume < abs_ofi) {
        s.volume = abs_ofi + 2;
    }
    if (((s.volume + s.ofi) & 1) != 0) {
        ++s.volume;
    }
}

std::vector<Sample> build_zero_ofi_sequence(int length) {
    std::vector<Sample> seq;
    seq.reserve(length);
    for (int i = 0; i < length; ++i) {
        Sample s{};
        s.ofi = 0;
        s.volume = 30 + (i % 4) * 4;
        s.log_return = 0.0005 * (static_cast<double>(i % 6) - 3.0);
        ensure_volume_alignment(s);
        seq.push_back(s);
    }
    return seq;
}

std::vector<Sample> build_negative_return_sequence(int length) {
    auto seq = build_sample_sequence(length);
    for (int i = 0; i < length; ++i) {
        seq[i].log_return = -0.001 * (static_cast<double>(i % 5) + 1.0);
        ensure_volume_alignment(seq[i]);
    }
    return seq;
}

std::vector<Sample> build_large_ofi_sequence(int length) {
    std::vector<Sample> seq;
    seq.reserve(length);
    for (int i = 0; i < length; ++i) {
        Sample s{};
        s.ofi = ((i % 6) - 3) * 20;
        s.volume = std::abs(s.ofi) + 40 + (i % 3) * 4;
        s.log_return = 0.0008 * (static_cast<double>((i * 2) % 7) - 3.0);
        ensure_volume_alignment(s);
        seq.push_back(s);
    }
    return seq;
}

std::vector<Sample> build_volume_spike_sequence(int length) {
    std::vector<Sample> seq;
    seq.reserve(length);
    for (int i = 0; i < length; ++i) {
        Sample s{};
        s.ofi = ((i % 5) - 2) * 6;
        s.volume = 40 + (i % 6) * 35;
        s.log_return = 0.0003 * (static_cast<double>(i % 9) - 4.0);
        ensure_volume_alignment(s);
        seq.push_back(s);
    }
    return seq;
}

struct SimulationResult {
    std::vector<double> reference;
    std::vector<double> actual;
};

SimulationResult run_sequence(GaussianCopulaFactor& factor,
                              ReferenceCopulaModel& reference,
                              const std::string& code,
                              const std::vector<Sample>& samples,
                              bool split_orders = false) {
    QuoteDepth init = make_quote(code, kStartTs - kQuoteStep, 100.0);
    factor.on_quote(init);

    std::vector<double> ref_values;
    double last_mid = 100.0;
    double mid = last_mid;
    int64_t ts = kStartTs;
    for (const auto& sample : samples) {
        feed_orders(factor, code, ts, sample.ofi, sample.volume, split_orders);
        double new_mid = mid * std::exp(sample.log_return);
        factor.on_quote(make_quote(code, ts + kQuoteStep, new_mid));

        double actual_ret = std::log(new_mid / last_mid);
        reference.push(static_cast<double>(sample.ofi),
                       static_cast<double>(sample.volume),
                       actual_ret);
        if (reference.is_window_full()) {
            ref_values.push_back(reference.predict(sample.ofi, sample.volume));
        }

        last_mid = new_mid;
        mid = new_mid;
        ts += 2 * kQuoteStep;
    }

    const auto scoped = compose_scope_code(code, static_cast<int>(reference.window_size()));
    return {ref_values, read_predictions(scoped)};
}

class GaussianCopulaFactorTest : public ::testing::Test {
protected:
    void SetUp() override {
        GaussianCopulaFactor::register_topics(2048);
    }
    BusGuard guard_;
};

void expect_equal_series(const std::vector<double>& expected,
                         const std::vector<double>& actual,
                         double tol = 1e-6) {
    ASSERT_EQ(expected.size(), actual.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        EXPECT_NEAR(actual[i], expected[i], tol) << "index=" << i;
    }
}

} // namespace

TEST_F(GaussianCopulaFactorTest, PredictionsMatchReferenceModel) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 6;
    cfg.regularization = 1e-6;

    const std::string code = "GC_MATCH";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_sample_sequence(24);

    auto result = run_sequence(factor, ref, code, seq, false);
    ASSERT_FALSE(result.reference.empty());
    expect_equal_series(result.reference, result.actual, 5e-7);
}

TEST_F(GaussianCopulaFactorTest, NoPredictionUntilWindowFilled) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 10;
    cfg.regularization = 1e-5;

    const std::string code = "GC_DELAY";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_sample_sequence(8); // shorter than window

    auto result = run_sequence(factor, ref, code, seq, false);
    EXPECT_TRUE(result.reference.empty());
    EXPECT_TRUE(result.actual.empty());
}

TEST_F(GaussianCopulaFactorTest, PendingOrdersAreAccumulatedAcrossEvents) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 5;
    cfg.regularization = 1e-6;

    const std::string code = "GC_PENDING";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_sample_sequence(18);

    auto result = run_sequence(factor, ref, code, seq, true);
    ASSERT_FALSE(result.reference.empty());
    expect_equal_series(result.reference, result.actual, 5e-7);
}

TEST_F(GaussianCopulaFactorTest, ForceFlushUsesPendingOrders) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 4;
    cfg.regularization = 1e-6;

    const std::string code = "GC_FLUSH";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_sample_sequence(12);

    auto result = run_sequence(factor, ref, code, seq, false);
    ASSERT_FALSE(result.actual.empty());
    auto scoped = compose_scope_code(code, static_cast<int>(ref.window_size()));
    const size_t before = result.actual.size();

    // 累积一笔新的 OFI/volume ，但不触发 quote
    const Sample pending{6, 20, 0.0};
    feed_orders(factor, code, kStartTs + 9999, pending.ofi, pending.volume, false);

    ASSERT_TRUE(ref.is_window_full());
    const double expected = ref.predict(pending.ofi, pending.volume);

    ASSERT_TRUE(factor.force_flush(scoped));
    auto after_values = read_predictions(scoped);
    ASSERT_EQ(before + 1, after_values.size());
    EXPECT_NEAR(after_values.back(), expected, 1e-6);
}

TEST_F(GaussianCopulaFactorTest, MultipleCodesStayIndependent) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 6;
    cfg.regularization = 1e-6;

    const std::string code_a = "GC_MULTI_A";
    const std::string code_b = "GC_MULTI_B";
    GaussianCopulaFactor factor(cfg, {code_a, code_b});
    ReferenceCopulaModel ref_a(cfg.window_size, cfg.regularization);
    ReferenceCopulaModel ref_b(cfg.window_size, cfg.regularization);

    auto seq_a = build_sample_sequence(20);
    auto seq_b = build_sample_sequence(22);

    auto res_a = run_sequence(factor, ref_a, code_a, seq_a, false);
    auto res_b = run_sequence(factor, ref_b, code_b, seq_b, true);

    ASSERT_FALSE(res_a.reference.empty());
    ASSERT_FALSE(res_b.reference.empty());
    expect_equal_series(res_a.reference, res_a.actual, 5e-7);
    expect_equal_series(res_b.reference, res_b.actual, 5e-7);
}

TEST_F(GaussianCopulaFactorTest, ZeroOFISequenceMatchesReference) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 5;
    cfg.regularization = 1e-6;

    const std::string code = "GC_ZERO_OFI";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_zero_ofi_sequence(24);

    auto result = run_sequence(factor, ref, code, seq, false);
    ASSERT_FALSE(result.reference.empty());
    expect_equal_series(result.reference, result.actual, 5e-7);
}

TEST_F(GaussianCopulaFactorTest, NegativeReturnSequenceMatchesReference) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 6;
    cfg.regularization = 1e-6;

    const std::string code = "GC_NEG_RET";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_negative_return_sequence(26);

    auto result = run_sequence(factor, ref, code, seq, true);
    ASSERT_FALSE(result.reference.empty());
    expect_equal_series(result.reference, result.actual, 3e-4);
}

TEST_F(GaussianCopulaFactorTest, HighRegularizationMatchesReference) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 7;
    cfg.regularization = 1e-3;

    const std::string code = "GC_HIGH_REG";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_sample_sequence(30);

    auto result = run_sequence(factor, ref, code, seq, false);
    ASSERT_FALSE(result.reference.empty());
    expect_equal_series(result.reference, result.actual, 5e-6);
}

TEST_F(GaussianCopulaFactorTest, ShortWindowMatchesReference) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 3;
    cfg.regularization = 1e-6;

    const std::string code = "GC_SHORT";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_sample_sequence(18);

    auto result = run_sequence(factor, ref, code, seq, false);
    ASSERT_FALSE(result.reference.empty());
    expect_equal_series(result.reference, result.actual, 5e-7);
}

TEST_F(GaussianCopulaFactorTest, LongSequenceMaintainsAlignment) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 6;
    cfg.regularization = 1e-6;

    const std::string code = "GC_LONG";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_sample_sequence(60);

    auto result = run_sequence(factor, ref, code, seq, false);
    ASSERT_FALSE(result.reference.empty());
    expect_equal_series(result.reference, result.actual, 5e-7);
}

TEST_F(GaussianCopulaFactorTest, SplitAndSingleOrdersProduceSamePredictions) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 6;
    cfg.regularization = 1e-6;

    auto seq = build_sample_sequence(32);

    const std::string code_single = "GC_SINGLE";
    GaussianCopulaFactor factor_single(cfg, {code_single});
    ReferenceCopulaModel ref_single(cfg.window_size, cfg.regularization);
    auto single = run_sequence(factor_single, ref_single, code_single, seq, false);
    expect_equal_series(single.reference, single.actual, 5e-7);

    const std::string code_split = "GC_SPLIT";
    GaussianCopulaFactor factor_split(cfg, {code_split});
    ReferenceCopulaModel ref_split(cfg.window_size, cfg.regularization);
    auto split = run_sequence(factor_split, ref_split, code_split, seq, true);
    expect_equal_series(split.reference, split.actual, 5e-7);

    ASSERT_EQ(single.actual.size(), split.actual.size());
    expect_equal_series(single.actual, split.actual, 5e-7);
}

TEST_F(GaussianCopulaFactorTest, ForceFlushBeforeWindowFullDoesNotPublish) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 10;
    cfg.regularization = 1e-6;

    const std::string code = "GC_FLUSH_EARLY";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_sample_sequence(6);

    auto result = run_sequence(factor, ref, code, seq, false);
    EXPECT_TRUE(result.reference.empty());
    EXPECT_TRUE(result.actual.empty());

    auto scoped = compose_scope_code(code, cfg.window_size);
    EXPECT_FALSE(factor.force_flush(scoped));
    auto after = read_predictions(scoped);
    EXPECT_TRUE(after.empty());
}

TEST_F(GaussianCopulaFactorTest, LargeOFISequenceMatchesReference) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 5;
    cfg.regularization = 1e-6;

    const std::string code = "GC_LARGE_OFI";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_large_ofi_sequence(28);

    auto result = run_sequence(factor, ref, code, seq, false);
    ASSERT_FALSE(result.reference.empty());
    expect_equal_series(result.reference, result.actual, 5e-7);
}

TEST_F(GaussianCopulaFactorTest, VolumeSpikeSequenceMatchesReference) {
    GaussianCopulaConfig cfg;
    cfg.window_size = 6;
    cfg.regularization = 1e-6;

    const std::string code = "GC_VOL_SPIKE";
    GaussianCopulaFactor factor(cfg, {code});
    ReferenceCopulaModel ref(cfg.window_size, cfg.regularization);
    auto seq = build_volume_spike_sequence(30);

    auto result = run_sequence(factor, ref, code, seq, false);
    ASSERT_FALSE(result.reference.empty());
    expect_equal_series(result.reference, result.actual, 5e-7);
}

TEST_F(GaussianCopulaFactorTest, DifferentWindowSizesProduceDifferentSeriesLengths) {
    GaussianCopulaConfig cfg_small;
    cfg_small.window_size = 4;
    cfg_small.regularization = 1e-6;

    GaussianCopulaConfig cfg_large = cfg_small;
    cfg_large.window_size = 9;

    auto seq = build_sample_sequence(40);

    const std::string code_small = "GC_WIN_SMALL";
    GaussianCopulaFactor factor_small(cfg_small, {code_small});
    ReferenceCopulaModel ref_small(cfg_small.window_size, cfg_small.regularization);
    auto res_small = run_sequence(factor_small, ref_small, code_small, seq, false);
    expect_equal_series(res_small.reference, res_small.actual, 5e-7);

    const std::string code_large = "GC_WIN_LARGE";
    GaussianCopulaFactor factor_large(cfg_large, {code_large});
    ReferenceCopulaModel ref_large(cfg_large.window_size, cfg_large.regularization);
    auto res_large = run_sequence(factor_large, ref_large, code_large, seq, false);
    expect_equal_series(res_large.reference, res_large.actual, 5e-7);

    ASSERT_GT(res_small.actual.size(), res_large.actual.size());
}
