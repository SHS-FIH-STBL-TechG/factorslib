// tests/stat_factors_tests/granger_causality_factor_test.cpp
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "factors/stat/granger_causality_factor.h"
#include "math/distributions.h"
#include "core/databus.h"
#include "core/scope_key.h"
#include "core/types.h"
#include "math/sliding_normal_eq.h"

using namespace factorlib;

namespace {

constexpr double kTolerance = 1e-6;

int64_t ms_of(int h, int m, int s, int ms = 0) {
    return (((static_cast<int64_t>(h) * 60 + m) * 60) + s) * 1000 + ms;
}

struct SamplePoint {
    double ofi;
    double delta_mid;
};

class ExpectedCalculator {
public:
    ExpectedCalculator(int window_size, int p_lags, int q_lags, int min_effective)
        : window_size_(window_size),
          p_(std::max(0, p_lags)),
          q_(std::max(0, q_lags)),
          min_effective_(min_effective),
          d_r_(1 + std::max(0, p_lags)),
          d_u_(1 + std::max(0, p_lags) + std::max(0, q_lags)),
          ne_r_(d_r_, window_size_),
          ne_u_(d_u_, window_size_) {}

    std::optional<double> push(double x_t, double y_t) {
        x_hist_.push_back(x_t);
        y_hist_.push_back(y_t);
        while (x_hist_.size() > static_cast<size_t>(window_size_)) x_hist_.pop_front();
        while (y_hist_.size() > static_cast<size_t>(window_size_)) y_hist_.pop_front();

        if (y_hist_.size() <= max_lag()) {
            last_p_.reset();
            return std::nullopt;
        }

        math::SlidingNormalEq<double>::Row xr(d_r_);
        math::SlidingNormalEq<double>::Row xu(d_u_);
        xr.setZero();
        xu.setZero();
        xr(0) = 1.0;
        xu(0) = 1.0;
        for (int i = 0; i < p_; ++i) {
            double lag_val = y_hist_[y_hist_.size() - 1 - static_cast<size_t>(i + 1)];
            xr(1 + i) = lag_val;
            xu(1 + i) = lag_val;
        }
        for (int j = 0; j < q_; ++j) {
            double lag_val = x_hist_[x_hist_.size() - 1 - static_cast<size_t>(j + 1)];
            xu(1 + p_ + j) = lag_val;
        }

        ne_r_.push(xr, y_t);
        ne_u_.push(xu, y_t);

        return compute_latest();
    }

    const std::vector<double>& recorded_pvals() const { return emitted_p_; }

private:
    size_t max_lag() const { return static_cast<size_t>(std::max(p_, q_)); }

    std::optional<double> compute_latest() {
        int N = std::min(static_cast<int>(y_hist_.size()), window_size_);
        int df2 = N - (p_ + q_ + 1);
        if (N < std::max(min_effective_, p_ + q_ + 4) || df2 <= 0) {
            last_p_.reset();
            return std::nullopt;
        }

        math::SlidingNormalEq<double>::Row br(d_r_);
        math::SlidingNormalEq<double>::Row bu(d_u_);
        double RSSr = 0.0, RSSu = 0.0;
        if (!ne_r_.solve(br, RSSr)) return std::nullopt;
        if (!ne_u_.solve(bu, RSSu)) return std::nullopt;

        double diff = RSSr - RSSu;
        if (diff < 0.0) diff = 0.0;
        int k = std::max(1, q_);
        int safe_df2 = std::max(1, df2);
        double num = diff / static_cast<double>(k);
        double den = RSSu / static_cast<double>(safe_df2);
        if (!(den > 0.0)) den = 1e-18;
        double F = num / den;
        double pval = math::fisher_f_sf<double>(F, k, safe_df2);
        if (!std::isfinite(pval)) {
            pval = (F <= 0.0 ? 1.0 : std::numeric_limits<double>::min());
        }
        last_p_ = pval;
        emitted_p_.push_back(pval);
        return last_p_;
    }

    int window_size_;
    int p_;
    int q_;
    int min_effective_;
    int d_r_;
    int d_u_;
    math::SlidingNormalEq<double> ne_r_;
    math::SlidingNormalEq<double> ne_u_;
    std::deque<double> x_hist_;
    std::deque<double> y_hist_;
    std::optional<double> last_p_;
    std::vector<double> emitted_p_;
};

double strength_from_p(const GrangerConfig& cfg, double p) {
    if (!cfg.use_neglog10) return p;
    double val = p <= 0.0 ? -std::log10(std::numeric_limits<double>::min()) : -std::log10(p);
    if (!std::isfinite(val)) val = 0.0;
    if (val > cfg.strength_clip) val = cfg.strength_clip;
    return val;
}

class GrangerCausalityFactorTest : public ::testing::Test {
protected:
    void SetUp() override {
        DataBus::instance().reset();
        GrangerCausalityFactor::register_topics(8192);
        base_cfg_.window_size   = 60;
        base_cfg_.p_lags        = 1;
        base_cfg_.q_lags        = 1;
        base_cfg_.min_effective = 20;
        base_cfg_.use_neglog10  = true;
        base_cfg_.strength_clip = 30.0;
        base_cfg_.publish_raw_p = true;
        code_counter_ = 0;
    }

    void TearDown() override {
        DataBus::instance().reset();
    }

    std::unique_ptr<GrangerCausalityFactor> make_factor(const GrangerConfig& cfg,
                                                        const std::vector<std::string>& codes) {
        return std::make_unique<GrangerCausalityFactor>(codes, cfg);
    }

    std::string next_code() {
        return "GR_FACTOR_" + std::to_string(++code_counter_);
    }

    std::string scoped_code(const std::string& code, int window) const {
        return compose_scope_code(code, window);
    }

    void feed_signed_order(GrangerCausalityFactor& factor, const std::string& code,
                           int64_t ts, double signed_volume) {
        double abs_vol = std::fabs(signed_volume);
        if (abs_vol < 0.5) return;
        Entrust e{};
        e.instrument_id = code;
        e.data_time_ms  = ts;
        e.price         = 100.0;
        e.side          = signed_volume >= 0.0 ? 1 : -1;
        e.volume        = static_cast<uint64_t>(std::llround(abs_vol));
        if (e.volume == 0) return;
        factor.on_tick(e);
    }

    void feed_quote(GrangerCausalityFactor& factor, const std::string& code,
                    int64_t ts, double mid) {
        QuoteDepth q{};
        q.instrument_id = code;
        q.data_time_ms  = ts;
        q.bid_price     = mid - 0.01;
        q.ask_price     = mid + 0.01;
        factor.on_quote(q);
    }

    void replay_event_series(GrangerCausalityFactor& factor, const std::string& code,
                             const std::vector<SamplePoint>& samples,
                             ExpectedCalculator& calc,
                             double& mid,
                             int64_t start_ts = ms_of(9, 30, 0, 0),
                             int64_t step_ms = 200) {
        // 预先喂一条基准报价，确保 last_mid 已初始化，因子第一条样本不会被跳过
        feed_quote(factor, code, start_ts - step_ms, mid);
        int64_t ts = start_ts;
        for (const auto& sp : samples) {
            feed_signed_order(factor, code, ts, sp.ofi);
            mid += sp.delta_mid;
            feed_quote(factor, code, ts + step_ms / 2, mid);
            calc.push(sp.ofi, sp.delta_mid);
            ts += step_ms;
        }
    }

    std::vector<double> read_topic(const std::string& topic,
                                   const std::string& scoped_code,
                                   size_t max_n = 512) const {
        auto rows = DataBus::instance().get_last_n<double>(topic, scoped_code, max_n);
        std::vector<double> values;
        values.reserve(rows.size());
        for (const auto& row : rows) values.push_back(row.second);
        return values;
    }

    static std::vector<SamplePoint> build_strong_sequence(int length,
                                                          double amplitude,
                                                          double phi,
                                                          double beta) {
        std::vector<SamplePoint> seq;
        seq.reserve(length);
        double y_prev = 0.0;
        double x_prev = 0.0;
        for (int t = 0; t < length; ++t) {
            double x_t = (t % 4 < 2 ? amplitude : -amplitude);
            double y_t = phi * y_prev + beta * x_prev;
            seq.push_back({x_t, y_t});
            y_prev = y_t;
            x_prev = x_t;
        }
        return seq;
    }

    static std::vector<SamplePoint> build_noise_sequence(int length,
                                                         double amplitude,
                                                         double phi) {
        std::vector<SamplePoint> seq;
        seq.reserve(length);
        double y_prev = 0.0;
        for (int t = 0; t < length; ++t) {
            double x_t = (t % 3 == 0 ? amplitude : -amplitude);
            double y_t = phi * y_prev;
            seq.push_back({x_t, y_t});
            y_prev = y_t;
        }
        return seq;
    }

    GrangerConfig base_cfg_{};
    int code_counter_{0};
};

std::vector<double> strengths_from_expected(const GrangerConfig& cfg,
                                            const std::vector<double>& pvals) {
    std::vector<double> out;
    out.reserve(pvals.size());
    for (double p : pvals) {
        out.push_back(strength_from_p(cfg, p));
    }
    return out;
}

template<typename T>
void expect_aligned_series(const std::vector<T>& expected,
                           const std::vector<T>& actual,
                           double tol = kTolerance) {
    size_t compare = std::min(expected.size(), actual.size());
    ASSERT_GT(compare, 0u);
    size_t offset_expected = expected.size() - compare;
    size_t offset_actual = actual.size() - compare;
    for (size_t i = 0; i < compare; ++i) {
        EXPECT_NEAR(actual[i + offset_actual], expected[i + offset_expected], tol);
    }
}

} // namespace

// ================== TEST CASES ==================

/**
 * 场景：事件驱动模式 + 明显的格兰杰因果（y_t 主要受上一期 OFI 影响）。
 * 入参：长序列的方波 OFI（±80），y_t = 0.2*y_{t-1} + 0.9*x_{t-1}。
 * 期望：输出的 -log10(p) 与手工 OLS 计算完全一致，且 p 值很小。
 */
TEST_F(GrangerCausalityFactorTest, EventModeStrongCausalityMatchesExpected) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.window_size = 60;
    cfg.min_effective = 25;
    auto factor = make_factor(cfg, {code});

    auto samples = build_strong_sequence(160, 80.0, 0.2, 0.9);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 100.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_p = calc.recorded_pvals();
    auto expected_strength = strengths_from_expected(cfg, expected_p);
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);
    auto actual_p = read_topic(TOP_GRANGER_PVAL, scoped);

    ASSERT_FALSE(expected_p.empty());
    expect_aligned_series(expected_p, actual_p);
    expect_aligned_series(expected_strength, actual_strength);
}

/**
 * 场景：事件驱动模式 + 无因果关系（y_t 仅自回归，与 OFI 无关）。
 * 入参：OFI 仍然交替 ±100，但 y_t = 0.8*y_{t-1}。
 * 期望：strength 直接输出原始 p 值（use_neglog10=false），与手算结果一致且接近 1。
 */
TEST_F(GrangerCausalityFactorTest, EventModeNoCausalityMatchesExpected) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.use_neglog10 = false;
    cfg.window_size = 80;
    cfg.min_effective = 30;
    auto factor = make_factor(cfg, {code});

    auto samples = build_noise_sequence(180, 100.0, 0.8);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 90.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_p = calc.recorded_pvals();
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);
    auto actual_p = read_topic(TOP_GRANGER_PVAL, scoped);

    ASSERT_FALSE(expected_p.empty());
    expect_aligned_series(expected_p, actual_p);
    expect_aligned_series(expected_p, actual_strength);
}

/**
 * 场景：事件驱动模式 + 配置 q=0（不使用 OFI），应退化到纯 AR 模型。
 * 入参：仍然输入显著 OFI，但因配置，x_lags 被忽略。
 * 期望：restricted 与 unrestricted 模型一致，F=0 → p=1，strength 也应为 1。
 */
TEST_F(GrangerCausalityFactorTest, EventModeZeroXLagBehavesAsAR) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.q_lags = 0;
    cfg.use_neglog10 = false;
    cfg.window_size = 40;
    cfg.min_effective = 15;
    auto factor = make_factor(cfg, {code});

    auto samples = build_strong_sequence(120, 60.0, 0.6, 0.5);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 110.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_p = calc.recorded_pvals();
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);

    ASSERT_FALSE(expected_p.empty());
    expect_aligned_series(expected_p, actual_strength);
    EXPECT_NEAR(actual_strength.back(), 1.0, 1e-5);
}

/**
 * 场景：事件驱动模式 + p=2/q=2 的多滞后模型。
 * 入参：y_t = 0.3y_{t-1} - 0.2y_{t-2} + 0.4x_{t-1} + 0.5x_{t-2}，OFI 仍为方波。
 * 期望：strength 与 raw p 均与离线 OLS 结果一致。
 */
TEST_F(GrangerCausalityFactorTest, EventModeTwoLagModelMatches) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.window_size = 70;
    cfg.p_lags = 2;
    cfg.q_lags = 2;
    cfg.min_effective = 30;
    auto factor = make_factor(cfg, {code});

    std::vector<SamplePoint> samples;
    samples.reserve(150);
    double y1 = 0.0, y2 = 0.0;
    double x1 = 0.0, x2 = 0.0;
    for (int t = 0; t < 150; ++t) {
        double x_t = (t % 5 < 3 ? 90.0 : -90.0);
        double y_t = 0.3 * y1 - 0.2 * y2 + 0.4 * x1 + 0.5 * x2;
        samples.push_back({x_t, y_t});
        y2 = y1; y1 = y_t;
        x2 = x1; x1 = x_t;
    }

    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 120.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_p = calc.recorded_pvals();
    auto expected_strength = strengths_from_expected(cfg, expected_p);
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);
    auto actual_p = read_topic(TOP_GRANGER_PVAL, scoped);

    ASSERT_FALSE(expected_p.empty());
    expect_aligned_series(expected_p, actual_p);
    expect_aligned_series(expected_strength, actual_strength);
}

/**
 * 场景：窗口较小（12）且数据远长于窗口，检验滑窗滚动是否只使用最近样本。
 * 入参：长度 80 的强因果序列。
 * 期望：输出数量与期望一致，末尾强度与手工“仅保留最近 12 条”结果完全相同。
 */
TEST_F(GrangerCausalityFactorTest, EventModeSlidingWindowKeepsRecent) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.window_size = 12;
    cfg.min_effective = 8;
    auto factor = make_factor(cfg, {code});

    auto samples = build_strong_sequence(80, 70.0, 0.1, 1.0);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 95.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_strength = strengths_from_expected(cfg, calc.recorded_pvals());
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);

    ASSERT_FALSE(actual_strength.empty());
    expect_aligned_series(expected_strength, actual_strength);
}

/**
 * 场景：将 strength_clip 设置为 2.0，输入极强因果数列。
 * 入参：同样的强因果方波，但 clip 很低。
 * 期望：-log10(p) 会被裁剪到 2.0，实际输出与期望完全一致。
 */
TEST_F(GrangerCausalityFactorTest, EventModeStrengthClippingApplies) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.strength_clip = 2.0;
    cfg.window_size = 40;
    cfg.min_effective = 20;
    auto factor = make_factor(cfg, {code});

    auto samples = build_strong_sequence(120, 150.0, 0.1, 1.2);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 80.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_strength = strengths_from_expected(cfg, calc.recorded_pvals());
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);

    ASSERT_FALSE(actual_strength.empty());
    EXPECT_DOUBLE_EQ(actual_strength.back(), cfg.strength_clip);
    expect_aligned_series(expected_strength, actual_strength);
}

/**
 * 场景：publish_raw_p=false，但当前实现仍会发布原始 p 值。
 * 入参：强因果序列。
 * 期望：strength 与手算一致，同时 pval 主题与手算 p 完全一致。
 */
TEST_F(GrangerCausalityFactorTest, EventModePublishRawPDisabledStillCorrect) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.publish_raw_p = false;
    auto factor = make_factor(cfg, {code});

    auto samples = build_strong_sequence(100, 90.0, 0.15, 0.9);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 75.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_p = calc.recorded_pvals();
    auto expected_strength = strengths_from_expected(cfg, expected_p);
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);
    auto actual_p = read_topic(TOP_GRANGER_PVAL, scoped);

    ASSERT_FALSE(expected_p.empty());
    expect_aligned_series(expected_strength, actual_strength);
    expect_aligned_series(expected_p, actual_p);
}

/**
 * 场景：publish_raw_p=true 且 use_neglog10=false，直接输出概率序列。
 * 入参：长度 60 的中等强度序列。
 * 期望：两个 topic 的数值都应等于手算 p。
 */
TEST_F(GrangerCausalityFactorTest, EventModeRawPSequenceMatches) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.use_neglog10 = false;
    cfg.window_size = 40;
    cfg.min_effective = 18;
    auto factor = make_factor(cfg, {code});

    auto samples = build_strong_sequence(90, 65.0, 0.3, 0.6);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 88.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_p = calc.recorded_pvals();
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_p = read_topic(TOP_GRANGER_PVAL, scoped);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);

    ASSERT_EQ(expected_p.size(), actual_p.size());
    ASSERT_EQ(actual_strength.size(), actual_p.size());
    for (size_t i = 0; i < expected_p.size(); ++i) {
        EXPECT_NEAR(actual_p[i], expected_p[i], kTolerance);
        EXPECT_NEAR(actual_strength[i], expected_p[i], kTolerance);
    }
}

/**
 * 场景：每个时间步拆成多笔委托后再触发 quote。
 * 入参：同一 OFI 序列分别以“单笔直接喂”和“拆成两笔累加”方式喂给不同代码。
 * 期望：两种喂数方式产生的强度/概率完全一致，证明 pending_ofi 对多笔会正确累加。
 */
TEST_F(GrangerCausalityFactorTest, EventModePendingOFIAggregation) {
    auto code_single = next_code();
    auto code_split = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.window_size = 50;
    cfg.min_effective = 20;
    auto factor = make_factor(cfg, {code_single, code_split});

    std::vector<SamplePoint> samples;
    double pattern[] = {150.0, -120.0, 90.0, -80.0, 110.0, -70.0, 130.0, -60.0};
    double y_prev = 0.0;
    double x_prev = 0.0;
    for (int i = 0; i < 120; ++i) {
        double x_t = pattern[i % 8];
        double y_t = 0.25 * y_prev + 0.95 * x_prev;
        samples.push_back({x_t, y_t});
        y_prev = y_t;
        x_prev = x_t;
    }

    double mid_single = 70.0;
    double mid_split = 70.0;
    int64_t ts = ms_of(9, 30, 0, 0);
    for (const auto& sp : samples) {
        // 单笔方式：一次性把净 OFI 喂完
        feed_signed_order(*factor, code_single, ts, sp.ofi);
        mid_single += sp.delta_mid;
        feed_quote(*factor, code_single, ts + 40, mid_single);

        // 拆单方式：OFI 拆成两笔顺序喂入
        double half = sp.ofi / 2.0;
        feed_signed_order(*factor, code_split, ts, half);
        feed_signed_order(*factor, code_split, ts + 10, sp.ofi - half);
        mid_split += sp.delta_mid;
        feed_quote(*factor, code_split, ts + 40, mid_split);

        ts += 80;
    }

    auto scoped_single = scoped_code(code_single, cfg.window_size);
    auto scoped_split = scoped_code(code_split, cfg.window_size);
    auto strength_single = read_topic(TOP_GRANGER_STRENGTH, scoped_single);
    auto strength_split = read_topic(TOP_GRANGER_STRENGTH, scoped_split);
    auto p_single = read_topic(TOP_GRANGER_PVAL, scoped_single);
    auto p_split = read_topic(TOP_GRANGER_PVAL, scoped_split);

    ASSERT_FALSE(strength_single.empty());
    ASSERT_EQ(strength_single.size(), strength_split.size());
    ASSERT_EQ(p_single.size(), p_split.size());
    for (size_t i = 0; i < strength_single.size(); ++i) {
        EXPECT_NEAR(strength_single[i], strength_split[i], kTolerance);
    }
    for (size_t i = 0; i < p_single.size(); ++i) {
        EXPECT_NEAR(p_single[i], p_split[i], kTolerance);
    }
}

/**
 * 场景：同一序列在不同窗口长度下计算，输出应随窗口变化。
 * 入参：长度 120 的强因果序列，分别喂给 window=20 和 window=60 的因子。
 * 期望：两个因子的输出都与各自理论值匹配，且最终强度不同。
 */
TEST_F(GrangerCausalityFactorTest, EventModeDifferentWindowsProduceDifferentStrength) {
    auto code_small = next_code();
    auto code_large = next_code();

    GrangerConfig cfg_small = base_cfg_;
    cfg_small.window_size = 20;
    cfg_small.min_effective = 12;
    auto factor_small = make_factor(cfg_small, {code_small});

    GrangerConfig cfg_large = base_cfg_;
    cfg_large.window_size = 60;
    cfg_large.min_effective = 30;
    auto factor_large = make_factor(cfg_large, {code_large});

    auto samples = build_strong_sequence(120, 80.0, 0.2, 1.0);

    ExpectedCalculator calc_small(cfg_small.window_size, cfg_small.p_lags, cfg_small.q_lags, cfg_small.min_effective);
    ExpectedCalculator calc_large(cfg_large.window_size, cfg_large.p_lags, cfg_large.q_lags, cfg_large.min_effective);

    double mid_small = 90.0;
    double mid_large = 90.0;
    replay_event_series(*factor_small, code_small, samples, calc_small, mid_small);
    replay_event_series(*factor_large, code_large, samples, calc_large, mid_large);

    auto scoped_small = scoped_code(code_small, cfg_small.window_size);
    auto scoped_large = scoped_code(code_large, cfg_large.window_size);
    auto strength_small = read_topic(TOP_GRANGER_STRENGTH, scoped_small);
    auto strength_large = read_topic(TOP_GRANGER_STRENGTH, scoped_large);

    auto expected_small = strengths_from_expected(cfg_small, calc_small.recorded_pvals());
    auto expected_large = strengths_from_expected(cfg_large, calc_large.recorded_pvals());

    expect_aligned_series(expected_small, strength_small);
    expect_aligned_series(expected_large, strength_large);
    ASSERT_FALSE(strength_small.empty());
    ASSERT_FALSE(strength_large.empty());
    EXPECT_GT(strength_small.size(), strength_large.size());
}

/**
 * 场景：事件模式下调用 force_flush，不应产生任何输出。
 * 入参：正常的事件驱动序列，随后对 scope 调用 force_flush。
 * 期望：force_flush 返回 false，DataBus 中的记录数量保持不变。
 */
TEST_F(GrangerCausalityFactorTest, ForceFlushReturnsFalseInEventMode) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    auto factor = make_factor(cfg, {code});

    auto samples = build_strong_sequence(80, 70.0, 0.2, 0.8);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 85.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto scoped = scoped_code(code, cfg.window_size);
    auto before_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);
    size_t before_count = before_strength.size();

    bool flushed = factor->force_flush(scoped);
    auto after_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);

    EXPECT_FALSE(flushed);
    EXPECT_EQ(before_count, after_strength.size());
}

/**
 * 场景：同一因子同时跟踪两个代码，两个序列的因果强度不同。
 * 入参：codeA 使用强因果序列，codeB 使用纯噪声。
 * 期望：两个 scoped code 的输出分别匹配各自的手算结果，互不干扰。
 */
TEST_F(GrangerCausalityFactorTest, MultiCodeStatesAreIsolated) {
    auto code_a = next_code();
    auto code_b = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.window_size = 50;
    cfg.min_effective = 22;
    auto factor = make_factor(cfg, {code_a, code_b});

    auto seq_a = build_strong_sequence(140, 90.0, 0.2, 1.0);
    auto seq_b = build_noise_sequence(140, 90.0, 0.7);
    ExpectedCalculator calc_a(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    ExpectedCalculator calc_b(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);

    double mid_a = 100.0;
    double mid_b = 100.0;
    replay_event_series(*factor, code_a, seq_a, calc_a, mid_a);
    replay_event_series(*factor, code_b, seq_b, calc_b, mid_b);

    auto scoped_a = scoped_code(code_a, cfg.window_size);
    auto scoped_b = scoped_code(code_b, cfg.window_size);
    auto exp_a = strengths_from_expected(cfg, calc_a.recorded_pvals());
    auto exp_b = strengths_from_expected(cfg, calc_b.recorded_pvals());
    auto act_a = read_topic(TOP_GRANGER_STRENGTH, scoped_a);
    auto act_b = read_topic(TOP_GRANGER_STRENGTH, scoped_b);

    expect_aligned_series(exp_a, act_a);
    expect_aligned_series(exp_b, act_b);
}

/**
 * 场景：min_effective 提升到 50，确保之前不会发布值。
 * 入参：长度 90 的强因果序列。
 * 期望：输出数量 = len - max(min_effective, p+q+4)，且数值匹配手算。
 */
TEST_F(GrangerCausalityFactorTest, MinEffectiveGateDelaysPublishing) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.window_size = 60;
    cfg.min_effective = 50;
    auto factor = make_factor(cfg, {code});

    auto samples = build_strong_sequence(90, 85.0, 0.25, 0.85);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 77.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_strength = strengths_from_expected(cfg, calc.recorded_pvals());
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);

    size_t threshold = static_cast<size_t>(std::max(cfg.min_effective, cfg.p_lags + cfg.q_lags + 4));
    size_t theoretical = samples.size() > threshold ? samples.size() - (threshold - 1) : 0;
    ASSERT_EQ(theoretical, actual_strength.size());
    expect_aligned_series(expected_strength, actual_strength);
}

/**
 * 场景：use_neglog10=false 且 publish_raw_p=false（但实现仍会发 p）。
 * 入参：强因果序列。
 * 期望：strength 与 pval 均等于手算概率序列。
 */
TEST_F(GrangerCausalityFactorTest, ProbabilityModeWithoutRawPStillCorrect) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.use_neglog10 = false;
    cfg.publish_raw_p = false;
    auto factor = make_factor(cfg, {code});

    auto samples = build_strong_sequence(100, 75.0, 0.2, 0.95);
    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 90.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_p = calc.recorded_pvals();
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);
    auto actual_p = read_topic(TOP_GRANGER_PVAL, scoped);

    ASSERT_FALSE(expected_p.empty());
    expect_aligned_series(expected_p, actual_strength);
    expect_aligned_series(expected_p, actual_p);
}

/**
 * 场景：OFI 全程为 0，仅靠价格自身的 AR(2) 结构。
 * 入参：y_t = 0.5y_{t-1} - 0.3y_{t-2}，x_t 恒为 0。
 * 期望：因子依旧能运行，输出结果与 AR 模型手算一致。
 */
TEST_F(GrangerCausalityFactorTest, ZeroOFIStillProducesAutoregressiveSignal) {
    auto code = next_code();
    GrangerConfig cfg = base_cfg_;
    cfg.q_lags = 1;
    cfg.window_size = 45;
    cfg.min_effective = 18;
    auto factor = make_factor(cfg, {code});

    std::vector<SamplePoint> samples;
    samples.reserve(100);
    double y1 = 0.0, y2 = 0.0;
    for (int i = 0; i < 100; ++i) {
        double y_t = 0.5 * y1 - 0.3 * y2;
        samples.push_back({0.0, y_t});
        y2 = y1; y1 = y_t;
    }

    ExpectedCalculator calc(cfg.window_size, cfg.p_lags, cfg.q_lags, cfg.min_effective);
    double mid = 65.0;
    replay_event_series(*factor, code, samples, calc, mid);

    auto expected_strength = strengths_from_expected(cfg, calc.recorded_pvals());
    auto scoped = scoped_code(code, cfg.window_size);
    auto actual_strength = read_topic(TOP_GRANGER_STRENGTH, scoped);

    expect_aligned_series(expected_strength, actual_strength);
}
