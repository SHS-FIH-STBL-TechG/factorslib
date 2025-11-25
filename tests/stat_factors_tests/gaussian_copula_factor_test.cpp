// tests/stat_factors_tests/gaussian_copula_factor_test.cpp
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "../../src/stat_factors/gaussian_copula_factor.h"
#include "math/distributions.h"
#include "math/incremental_covariance.h"
#include "math/incremental_rank.h"
#include "math/linear_algebra.h"
#include "utils/databus.h"
#include "utils/processing_axes.h"
#include "utils/scope_key.h"
#include "utils/types.h"

using namespace factorlib;

namespace {

constexpr const char* kPredictionTopic = "gaussian_copula/prediction";
constexpr double kDefaultSpread = 0.02;
constexpr double kTolerance = 1e-9;

struct ScenarioSeries {
    std::vector<int> ofi;
    std::vector<int> volume;
    std::vector<double> log_returns;
};

std::string default_scope_code(const std::string& code, int window_size) {
    const auto& freqs = get_time_frequencies();
    int64_t freq = freqs.empty() ? 1 : freqs.front();
    return compose_scope_code(code, freq, window_size);
}

void assert_series_size(const ScenarioSeries& series) {
    ASSERT_EQ(series.ofi.size(), series.volume.size());
    ASSERT_EQ(series.ofi.size(), series.log_returns.size());
}

struct SimulationState {
    explicit SimulationState(size_t w)
        : window_size(w), cov(w) {}

    bool ingest(double ofi, double volume, double ret) {
        ofi_rank.push(ofi, window_size);
        volume_rank.push(volume, window_size);
        return_rank.push(ret, window_size);

        if (ofi_rank.size() >= window_size &&
            volume_rank.size() >= window_size &&
            return_rank.size() >= window_size) {
            double ofi_rank_val = ofi_rank.median_rank(ofi);
            double volume_rank_val = volume_rank.median_rank(volume);
            double return_rank_val = return_rank.median_rank(ret);

            double z_ofi = math::Distributions<double>::normal_quantile(ofi_rank_val);
            double z_volume = math::Distributions<double>::normal_quantile(volume_rank_val);
            double z_return = math::Distributions<double>::normal_quantile(return_rank_val);

            Eigen::Vector3d normal_score;
            normal_score << z_ofi, z_volume, z_return;
            cov.push(normal_score);
        }
        return window_full();
    }

    bool window_full() const {
        return ofi_rank.size() >= window_size &&
               volume_rank.size() >= window_size &&
               return_rank.size() >= window_size &&
               cov.size() >= window_size;
    }

    double predict(double current_ofi, double current_volume, double regularization) const {
        Eigen::Vector3d mean = cov.mean();
        Eigen::Matrix3d covariance = cov.covariance();
        covariance += Eigen::Matrix3d::Identity() * regularization;

        double current_ofi_rank = ofi_rank.median_rank(current_ofi);
        double current_volume_rank = volume_rank.median_rank(current_volume);

        double z_ofi_current = math::Distributions<double>::normal_quantile(current_ofi_rank);
        double z_volume_current = math::Distributions<double>::normal_quantile(current_volume_rank);

        Eigen::Vector2d conditions;
        conditions << z_ofi_current, z_volume_current;

        double conditional_mean = math::LinearAlgebra<double>::conditional_expectation(
            mean, covariance, conditions, 2);
        double conditional_probability = 0.5 * (1.0 + std::erf(conditional_mean / std::sqrt(2.0)));

        auto sorted_returns = return_rank.get_sorted_data();
        return math::Distributions<double>::empirical_inverse_cdf(sorted_returns,
                                                                  conditional_probability);
    }

    size_t window_size;
    math::IncrementalRankCalculator<double> ofi_rank;
    math::IncrementalRankCalculator<double> volume_rank;
    math::IncrementalRankCalculator<double> return_rank;
    math::IncrementalCovariance<double, 3> cov;
};

std::vector<double> compute_expected_sequence(const ScenarioSeries& series,
                                              size_t window_size,
                                              double regularization) {
    assert_series_size(series);
    SimulationState state(window_size);
    std::vector<double> expected;
    for (size_t i = 0; i < series.ofi.size(); ++i) {
        bool full = state.ingest(series.ofi[i], series.volume[i], series.log_returns[i]);
        if (full) {
            expected.push_back(state.predict(series.ofi[i], series.volume[i], regularization));
        }
    }
    return expected;
}

double compute_expected_with_override(const ScenarioSeries& series,
                                      size_t window_size,
                                      double regularization,
                                      double current_ofi,
                                      double current_volume) {
    assert_series_size(series);
    SimulationState state(window_size);
    for (size_t i = 0; i < series.ofi.size(); ++i) {
        state.ingest(series.ofi[i], series.volume[i], series.log_returns[i]);
    }
    EXPECT_TRUE(state.window_full());
    return state.predict(current_ofi, current_volume, regularization);
}

ScenarioSeries slice_series(const ScenarioSeries& series, size_t start_index) {
    assert_series_size(series);
    if (start_index > series.ofi.size()) {
        ADD_FAILURE() << "start_index=" << start_index
                      << " 大于序列长度=" << series.ofi.size();
        return {};
    }
    ScenarioSeries out;
    out.ofi.assign(series.ofi.begin() + static_cast<std::ptrdiff_t>(start_index), series.ofi.end());
    out.volume.assign(series.volume.begin() + static_cast<std::ptrdiff_t>(start_index), series.volume.end());
    out.log_returns.assign(series.log_returns.begin() + static_cast<std::ptrdiff_t>(start_index), series.log_returns.end());
    return out;
}

int64_t ms_of(int h, int m, int s, int ms = 0) {
    return (((static_cast<int64_t>(h) * 60 + m) * 60) + s) * 1000 + ms;
}

void emit_order(GaussianCopulaFactor& factor,
                const std::string& code,
                int side,
                uint64_t volume,
                int64_t ts,
                uint64_t& seq) {
    Entrust e{};
    e.instrument_id = code;
    e.data_time_ms = ts;
    e.price = 100.0;
    e.side = side;
    e.volume = volume;
    e.main_seq = ++seq;
    e.order_id = ++seq;
    factor.on_tick(e);
}

void apply_order_flow_impl(GaussianCopulaFactor& factor,
                           const std::string& code,
                           int ofi_value,
                           int volume_value,
                           int64_t ts,
                           uint64_t& seq) {
    ASSERT_GE(volume_value, 0);
    ASSERT_GE(volume_value, std::abs(ofi_value));
    double buy = (static_cast<double>(volume_value) + static_cast<double>(ofi_value)) / 2.0;
    double sell = (static_cast<double>(volume_value) - static_cast<double>(ofi_value)) / 2.0;

    auto buy_volume = static_cast<uint64_t>(std::llround(std::max(0.0, buy)));
    auto sell_volume = static_cast<uint64_t>(std::llround(std::max(0.0, sell)));
    int realized_ofi = static_cast<int>(buy_volume) - static_cast<int>(sell_volume);
    int realized_volume = static_cast<int>(buy_volume) + static_cast<int>(sell_volume);
    ASSERT_EQ(realized_ofi, ofi_value);
    ASSERT_EQ(realized_volume, volume_value);

    if (buy_volume > 0) {
        emit_order(factor, code, +1, buy_volume, ts, seq);
    }
    if (sell_volume > 0) {
        emit_order(factor, code, -1, sell_volume, ts, seq);
    }
}

void run_series_impl(GaussianCopulaFactor& factor,
                     const std::string& code,
                     const ScenarioSeries& series,
                     uint64_t& seq,
                     double initial_mid,
                     int64_t start_ts,
                     int64_t step_ms) {
    assert_series_size(series);
    QuoteDepth q{};
    q.instrument_id = code;
    q.data_time_ms = start_ts;
    q.bid_price = initial_mid - kDefaultSpread / 2.0;
    q.ask_price = initial_mid + kDefaultSpread / 2.0;
    factor.on_quote(q);

    double mid = initial_mid;
    int64_t ts = start_ts;
    for (size_t i = 0; i < series.ofi.size(); ++i) {
        ts += step_ms / 2;
        apply_order_flow_impl(factor, code, series.ofi[i], series.volume[i], ts, seq);
        ts += step_ms / 2;
        mid *= std::exp(series.log_returns[i]);
        QuoteDepth next = q;
        next.data_time_ms = ts;
        next.bid_price = mid - kDefaultSpread / 2.0;
        next.ask_price = mid + kDefaultSpread / 2.0;
        factor.on_quote(next);
    }
}

class GaussianCopulaFactorTest : public ::testing::Test {
protected:
    void SetUp() override {
        DataBus::instance().reset();
        GaussianCopulaFactor::register_topics(4096);
        base_cfg_.window_size = 3;
        base_cfg_.regularization = 1e-6;
        test_code_ = make_code("GC");
        factor_ = std::make_unique<GaussianCopulaFactor>(base_cfg_, std::vector<std::string>{test_code_});
        order_seq_ = 0;
    }

    void TearDown() override {
        factor_.reset();
    }

    std::string make_code(const std::string& prefix) {
        return prefix + "_" + std::to_string(++code_counter_);
    }

    std::vector<double> read_predictions(const std::string& code, size_t max_n = 64) const {
        auto rows = DataBus::instance().get_last_n<double>(kPredictionTopic, code, max_n);
        std::vector<double> values;
        values.reserve(rows.size());
        for (const auto& row : rows) {
            values.push_back(row.second);
        }
        return values;
    }

    std::string scoped_code_for_test() const {
        return default_scope_code(test_code_, base_cfg_.window_size);
    }

    void run_series(const ScenarioSeries& series) {
        run_series_impl(*factor_, test_code_, series, order_seq_, 100.0, ms_of(9, 30, 0, 0), 1000);
    }

    void run_series_with(GaussianCopulaFactor& factor,
                         const std::string& code,
                         const ScenarioSeries& series,
                         uint64_t& seq,
                         double initial_mid = 100.0,
                         int64_t start_ts = ms_of(9, 30, 0, 0),
                         int64_t step_ms = 1000) {
        run_series_impl(factor, code, series, seq, initial_mid, start_ts, step_ms);
    }

    void apply_order_flow(int ofi, int volume, int64_t ts) {
        apply_order_flow_impl(*factor_, test_code_, ofi, volume, ts, order_seq_);
    }

    std::unique_ptr<GaussianCopulaFactor> make_factor(const GaussianCopulaConfig& cfg,
                                                      const std::vector<std::string>& codes) {
        return std::make_unique<GaussianCopulaFactor>(cfg, codes);
    }

    GaussianCopulaConfig base_cfg_{};
    std::string test_code_;
    std::unique_ptr<GaussianCopulaFactor> factor_;
    uint64_t order_seq_{0};
    static int code_counter_;
};

int GaussianCopulaFactorTest::code_counter_ = 0;

// 场景：窗口=3且买卖力量完全对称，验证完全中性的输入不会产生虚假的收益预测。
// 入参：5个样本的OFI序列为{20,-20,20,-20,20}，成交量均为40，价格无波动（log return为0）。
// 期望：只能发布1个预测值，且该值接近0并与手动预期完全一致。
TEST_F(GaussianCopulaFactorTest, SymmetricFlowFlatReturnsYieldZero) {
    ScenarioSeries series{
        {20, -20, 20, -20, 20},
        {40, 40, 40, 40, 40},
        {0.0, 0.0, 0.0, 0.0, 0.0}
    };

    run_series(series);
    auto actual = read_predictions(test_code_);
    auto expected = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 1u);
    EXPECT_NEAR(actual[0], expected[0], kTolerance);
    EXPECT_NEAR(actual[0], 0.0, kTolerance);
}

// 场景：持续的正向OFI与稳定上涨的收益率配合，用于确认因子在典型做多情形下输出正值。
// 入参：7个样本，OFI递增为{20,40,...,140}，成交量递增，收益率从0.001到0.0022。
// 期望：连续3个预测值均为正，并且逐个匹配手动计算结果。
TEST_F(GaussianCopulaFactorTest, PositiveFlowInUptrendProducesPositivePrediction) {
    ScenarioSeries series{
        {20, 40, 60, 80, 100, 120, 140},
        {60, 80, 100, 120, 140, 160, 180},
        {0.0010, 0.0012, 0.0014, 0.0016, 0.0018, 0.0020, 0.0022}
    };

    run_series(series);
    auto actual = read_predictions(test_code_);
    auto expected = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 3u);
    for (size_t i = 0; i < actual.size(); ++i) {
        EXPECT_NEAR(actual[i], expected[i], kTolerance);
        EXPECT_GT(actual[i], 0.0);
    }
}

// 场景：持续的负向OFI与下跌收益率配合，用于验证做空情形能给出负期望。
// 入参：7个样本，OFI为{-20,-40,...,-140}，成交量与正向样本一致，收益率为负。
// 期望：预测值应全部为负且与手工计算完全吻合。
TEST_F(GaussianCopulaFactorTest, NegativeFlowInDowntrendProducesNegativePrediction) {
    ScenarioSeries series{
        {-20, -40, -60, -80, -100, -120, -140},
        {60, 80, 100, 120, 140, 160, 180},
        {-0.0010, -0.0012, -0.0014, -0.0016, -0.0018, -0.0020, -0.0022}
    };

    run_series(series);
    auto actual = read_predictions(test_code_);
    auto expected = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 3u);
    for (size_t i = 0; i < actual.size(); ++i) {
        EXPECT_NEAR(actual[i], expected[i], kTolerance);
        EXPECT_LT(actual[i], 0.0);
    }
}

// 场景：OFI恒为0但成交量随时间放大，验证成交量维度单独提供信号的能力。
// 入参：6个样本，OFI为0，成交量递增为{40,60,...,140}，收益率逐步升高。
// 期望：在窗口满足后产生2个正值预测，并与理论计算一致。
TEST_F(GaussianCopulaFactorTest, ZeroOFIButRisingVolumeProvidesSignal) {
    ScenarioSeries series{
        {0, 0, 0, 0, 0, 0},
        {40, 60, 80, 100, 120, 140},
        {0.0004, 0.0007, 0.0010, 0.0013, 0.0016, 0.0020}
    };

    run_series(series);
    auto actual = read_predictions(test_code_);
    auto expected = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 2u);
    for (size_t i = 0; i < actual.size(); ++i) {
        EXPECT_NEAR(actual[i], expected[i], kTolerance);
        EXPECT_GT(actual[i], 0.0);
    }
}

// 场景：OFI与收益率正负交替，检验算法在噪声场景下是否给出接近0的中性预测。
// 入参：6个样本，OFI在±60之间交替，收益率以同样节奏正负跳动。
// 期望：窗口充满后的两个预测值都逼近0，且与手工计算一致。
TEST_F(GaussianCopulaFactorTest, AlternatingOrderFlowKeepsPredictionNearZero) {
    ScenarioSeries series{
        {60, -60, 50, -50, 40, -40},
        {80, 80, 80, 80, 80, 80},
        {0.0010, -0.0010, 0.0008, -0.0008, 0.0006, -0.0006}
    };

    run_series(series);
    auto actual = read_predictions(test_code_);
    auto expected = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 2u);
    for (size_t i = 0; i < actual.size(); ++i) {
        EXPECT_NEAR(actual[i], expected[i], kTolerance);
        EXPECT_NEAR(actual[i], 0.0, 1e-3);
    }
}

// 场景：滑动窗口应只保留最近样本，验证丢掉前两个数据后得到的预测与完整序列最后一次预测一致。
// 入参：7个样本，OFI递增{40,...,160}，收益率从0.0008升到0.0014。
// 期望：完整序列与裁剪序列（丢掉前两个样本）在末尾预测上完全一致。
TEST_F(GaussianCopulaFactorTest, SlidingWindowKeepsOnlyRecentSamples) {
    ScenarioSeries series{
        {40, 60, 80, 100, 120, 140, 160},
        {80, 100, 120, 140, 160, 180, 200},
        {0.0008, 0.0009, 0.0010, 0.0011, 0.0012, 0.0013, 0.0014}
    };
    run_series(series);
    auto full_preds = read_predictions(test_code_);
    auto expected_full = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(full_preds.size(), expected_full.size());
    ASSERT_EQ(full_preds.size(), 3u);

    ScenarioSeries trimmed = slice_series(series, 2);
    auto trim_code = make_code("TRIM");
    auto trim_factor = make_factor(base_cfg_, {trim_code});
    uint64_t trim_seq = 0;
    run_series_with(*trim_factor, trim_code, trimmed, trim_seq);
    auto trimmed_preds = read_predictions(trim_code);
    auto expected_trim = compute_expected_sequence(trimmed, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(trimmed_preds.size(), expected_trim.size());
    ASSERT_EQ(trimmed_preds.size(), 1u);

    EXPECT_NEAR(full_preds.back(), trimmed_preds.back(), kTolerance);
    EXPECT_NEAR(full_preds.back(), expected_trim.back(), kTolerance);
}

// 场景：较长序列生成多次预测，检验每次发布是否都与手动运算的全序列结果一致。
// 入参：8个样本，OFI在{40,20,60,30,...,50}之间波动，对应收益率亦交替。
// 期望：得到4个预测值，并且每一个都与预期值逐一相等。
TEST_F(GaussianCopulaFactorTest, PredictionSequenceMatchesManualComputation) {
    ScenarioSeries series{
        {40, 20, 60, 30, 80, 40, 100, 50},
        {80, 60, 100, 80, 120, 100, 140, 120},
        {0.0008, 0.0002, 0.0011, 0.0005, 0.0014, 0.0008, 0.0017, 0.0010}
    };

    run_series(series);
    auto actual = read_predictions(test_code_);
    auto expected = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 4u);
    for (size_t i = 0; i < actual.size(); ++i) {
        EXPECT_NEAR(actual[i], expected[i], kTolerance);
    }
}

// 场景：窗口填满后追加新的买单但尚未出新行情，验证force_flush利用最新OFI重新计算。
// 入参：5个样本填充窗口，随后追加净买OFI=200、成交量=200。
// 期望：force_flush发布的最新值与“旧窗口状态+新OFI”手工计算一致。
TEST_F(GaussianCopulaFactorTest, ForceFlushAdoptsLatestOrderFlow) {
    ScenarioSeries series{
        {30, 50, 70, 90, 110},
        {60, 80, 100, 120, 140},
        {0.0009, 0.0010, 0.0011, 0.0012, 0.0013}
    };

    run_series(series);
    apply_order_flow(200, 200, ms_of(10, 0, 0, 0));
    ASSERT_TRUE(factor_->force_flush(scoped_code_for_test()));

    auto actual = read_predictions(test_code_);
    auto expected_flush = compute_expected_with_override(series,
                                                         base_cfg_.window_size,
                                                         base_cfg_.regularization,
                                                         200.0,
                                                         200.0);
    EXPECT_NEAR(actual.back(), expected_flush, kTolerance);
}

// 场景：追加了成交量巨大但OFI为0的新订单，验证force_flush只因成交量变化而改变输出。
// 入参：5个样本后追加OFI=0、成交量=400的撮合。
// 期望：force_flush的结果等于将当前OFI=0、当前成交量=400代入算法得到的数值。
TEST_F(GaussianCopulaFactorTest, ForceFlushAdoptsLatestVolumeEvenWithoutOFI) {
    ScenarioSeries series{
        {40, -40, 60, -60, 80},
        {80, 80, 100, 100, 120},
        {0.0008, -0.0008, 0.0010, -0.0010, 0.0012}
    };

    run_series(series);
    apply_order_flow(0, 400, ms_of(10, 30, 0, 0));
    ASSERT_TRUE(factor_->force_flush(scoped_code_for_test()));

    auto actual = read_predictions(test_code_);
    auto expected_flush = compute_expected_with_override(series,
                                                         base_cfg_.window_size,
                                                         base_cfg_.regularization,
                                                         0.0,
                                                         400.0);
    EXPECT_NEAR(actual.back(), expected_flush, kTolerance);
}

// 场景：调大正则化参数应让预测值更“保守”，检验不同regularization下的精确数值。
// 入参：7个样本，分别用regularization=1e-6与1e-2进行对比。
// 期望：两条曲线的末位预测都与对应的手工结果相等，并且两者数值不同。
TEST_F(GaussianCopulaFactorTest, RegularizationParameterChangesPredictionMagnitude) {
    ScenarioSeries series{
        {20, 60, 40, 80, 60, 100, 80},
        {80, 120, 100, 140, 120, 160, 140},
        {0.0005, 0.0009, 0.0007, 0.0011, 0.0009, 0.0013, 0.0011}
    };

    GaussianCopulaConfig cfg_low = base_cfg_;
    cfg_low.regularization = 1e-6;
    GaussianCopulaConfig cfg_high = base_cfg_;
    cfg_high.regularization = 1e-2;

    auto code_low = make_code("LOW");
    auto code_high = make_code("HIGH");
    auto factor_low = make_factor(cfg_low, {code_low});
    auto factor_high = make_factor(cfg_high, {code_high});
    uint64_t seq_low = 0, seq_high = 0;

    run_series_with(*factor_low, code_low, series, seq_low);
    run_series_with(*factor_high, code_high, series, seq_high);

    auto actual_low = read_predictions(code_low);
    auto actual_high = read_predictions(code_high);
    auto expected_low = compute_expected_sequence(series, cfg_low.window_size, cfg_low.regularization);
    auto expected_high = compute_expected_sequence(series, cfg_high.window_size, cfg_high.regularization);

    ASSERT_FALSE(actual_low.empty());
    ASSERT_FALSE(actual_high.empty());
    EXPECT_NEAR(actual_low.back(), expected_low.back(), kTolerance);
    EXPECT_NEAR(actual_high.back(), expected_high.back(), kTolerance);
    EXPECT_GT(std::abs(actual_low.back() - actual_high.back()), 1e-7);
}

// 场景：将窗口长度改为4，验证需要更多样本才能发布且结果仍能匹配理论。
// 入参：7个样本填满窗口（2*4-1=7），OFI递增。
// 期望：只产生1个预测值，且与手工计算一致。
TEST_F(GaussianCopulaFactorTest, SupportsLongerWindowLengths) {
    GaussianCopulaConfig cfg = base_cfg_;
    cfg.window_size = 4;
    auto code = make_code("WIN4");
    factor_ = make_factor(cfg, {code});
    test_code_ = code;
    order_seq_ = 0;

    ScenarioSeries series{
        {40, 60, 80, 100, 120, 140, 160},
        {100, 120, 140, 160, 180, 200, 220},
        {0.0006, 0.0008, 0.0010, 0.0012, 0.0014, 0.0016, 0.0018}
    };

    run_series_with(*factor_, code, series, order_seq_);
    auto actual = read_predictions(code);
    auto expected = compute_expected_sequence(series, cfg.window_size, cfg.regularization);

    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 1u);
    EXPECT_NEAR(actual[0], expected[0], kTolerance);
}

// 场景：成交量和OFI都放到百万级别，确认不会因为大数值而溢出或失真。
// 入参：5个样本，OFI在±200000~300000之间变化，收益率正负交替。
// 期望：单个预测值与手动结果一致，不存在NaN或Inf。
TEST_F(GaussianCopulaFactorTest, HandlesLargeVolumesWithoutOverflow) {
    ScenarioSeries series{
        {200000, -200000, 250000, -250000, 300000},
        {400000, 400000, 500000, 500000, 600000},
        {0.0005, -0.0005, 0.0007, -0.0007, 0.0010}
    };

    run_series(series);
    auto actual = read_predictions(test_code_);
    auto expected = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 1u);
    EXPECT_FALSE(std::isnan(actual[0]));
    EXPECT_FALSE(std::isinf(actual[0]));
    EXPECT_NEAR(actual[0], expected[0], kTolerance);
}

// 场景：一个因子同时跟踪两个代码，验证不同代码的状态完全隔离。
// 入参：codeA与codeB分别喂入不同的6个样本序列。
// 期望：两个代码各自产生的预测值都与对应序列的手动结果相等，互不干扰。
TEST_F(GaussianCopulaFactorTest, DifferentCodesHaveIsolatedStates) {
    auto code_a = make_code("A");
    auto code_b = make_code("B");
    factor_ = make_factor(base_cfg_, {code_a, code_b});
    order_seq_ = 0;

    ScenarioSeries series_a{
        {40, 60, 80, 100, 120, 140},
        {80, 100, 120, 140, 160, 180},
        {0.0008, 0.0010, 0.0012, 0.0014, 0.0016, 0.0018}
    };
    ScenarioSeries series_b{
        {-30, -50, -70, -90, -110, -130},
        {90, 110, 130, 150, 170, 190},
        {-0.0007, -0.0009, -0.0011, -0.0013, -0.0015, -0.0017}
    };

    run_series_impl(*factor_, code_a, series_a, order_seq_, 100.0, ms_of(9, 30, 0, 0), 1000);
    run_series_impl(*factor_, code_b, series_b, order_seq_, 100.0, ms_of(9, 30, 0, 5000), 1000);

    auto preds_a = read_predictions(code_a);
    auto preds_b = read_predictions(code_b);
    auto expected_a = compute_expected_sequence(series_a, base_cfg_.window_size, base_cfg_.regularization);
    auto expected_b = compute_expected_sequence(series_b, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(preds_a.size(), expected_a.size());
    ASSERT_EQ(preds_b.size(), expected_b.size());
    for (size_t i = 0; i < preds_a.size(); ++i) {
        EXPECT_NEAR(preds_a[i], expected_a[i], kTolerance);
    }
    for (size_t i = 0; i < preds_b.size(); ++i) {
        EXPECT_NEAR(preds_b[i], expected_b[i], kTolerance);
    }
}

// 场景：OFI与成交量都为0但收益率上下跳动，检查算法是否正确回归到收益率分布的中位数。
// 入参：5个样本的OFI=0、成交量=0，收益率在正负之间交替。
// 期望：唯一的预测值接近收益率分布的中位数，并与手工计算一致。
TEST_F(GaussianCopulaFactorTest, ZeroOrderFlowWithMovingReturns) {
    ScenarioSeries series{
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0.0010, -0.0010, 0.0005, -0.0005, 0.0002}
    };

    run_series(series);
    auto actual = read_predictions(test_code_);
    auto expected = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);

    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 1u);
    EXPECT_NEAR(actual[0], expected[0], kTolerance);
}

// 场景：在每个撮合后额外插入CombinedTick类型为Trade的逐笔，验证这些噪声不会影响OFI计算。
// 入参：5个样本，OFI递增，且每次在成交量输入后插入Trade类型的CombinedTick。
// 期望：预测值与未插入Trade时的手工预期一致，证明Trade tick被正确忽略。
TEST_F(GaussianCopulaFactorTest, TradeTicksAreIgnoredByOrderFlowLogic) {
    ScenarioSeries series{
        {40, 60, 80, 100, 120},
        {80, 100, 120, 140, 160},
        {0.0009, 0.0010, 0.0011, 0.0012, 0.0013}
    };
    assert_series_size(series);

    QuoteDepth q{};
    q.instrument_id = test_code_;
    q.data_time_ms = ms_of(9, 30, 0, 0);
    q.bid_price = 100.0 - kDefaultSpread / 2.0;
    q.ask_price = 100.0 + kDefaultSpread / 2.0;
    factor_->on_quote(q);

    double mid = 100.0;
    int64_t ts = q.data_time_ms;
    for (size_t i = 0; i < series.ofi.size(); ++i) {
        ts += 400;
        apply_order_flow(series.ofi[i], series.volume[i], ts);
        CombinedTick trade{};
        trade.instrument_id = test_code_;
        trade.data_time_ms = ts + 10;
        trade.price = mid;
        trade.side = 1;
        trade.volume = 10;
        trade.kind = CombinedKind::Trade;
        factor_->on_tick(trade);
        ts += 400;
        mid *= std::exp(series.log_returns[i]);
        QuoteDepth next = q;
        next.data_time_ms = ts;
        next.bid_price = mid - kDefaultSpread / 2.0;
        next.ask_price = mid + kDefaultSpread / 2.0;
        factor_->on_quote(next);
    }

    auto actual = read_predictions(test_code_);
    auto expected = compute_expected_sequence(series, base_cfg_.window_size, base_cfg_.regularization);
    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual.size(), 1u);
    EXPECT_NEAR(actual[0], expected[0], kTolerance);
}

} // namespace
