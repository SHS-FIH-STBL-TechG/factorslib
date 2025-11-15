// tests/stat_factors_tests/probability_momentum_factor_test.cpp

#include "../../src/stat_factors/probability_momentum_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "utils/databus.h"
#include "utils/types.h"
#include "../utils/test_config.h"   // 使用 testcfg::read_*_from_cfg

using namespace factorlib;

namespace {

// 每个用例前后清理 DataBus，避免状态串扰
struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

// 取某个 code 的最新概率 / 信号
bool last_prob(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_PROB_MOM_PROB, code, out, ts);
}

bool last_signal(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_PROB_MOM_SIGNAL, code, out, ts);
}

} // namespace

// ============== 用例 1：Bar CSV 驱动（K 线，带期望值对比） ==============
TEST(ProbabilityMomentum, BarsCsvFeed_ExpectationMatches) {
    BusGuard _guard;
    ProbabilityMomentumFactor::register_topics(2048);

    // 从 test_config.ini 的 [global].bars_csv 读取 Bar 序列
    auto bars = testcfg::read_bars_from_cfg();
    ASSERT_FALSE(bars.empty()) << "bars_csv 未配置或无有效数据";
    ASSERT_GE(bars.size(), 40u) << "Bar CSV 行数太少（请至少保证 >= 40 行）";

    const std::string code = bars[0].instrument_id;

    ProbMomentumConfig cfg;
    cfg.window_size      = 30;
    cfg.min_total_weight = 10.0;
    cfg.min_sigma        = 1e-6;
    cfg.buy_threshold    = 0.7;
    cfg.sell_threshold   = 0.3;
    cfg.debug_mode       = false;

    ProbabilityMomentumFactor factor({ code }, cfg);

    for (const auto& b : bars) {
        factor.on_bar(b);
    }

    double F = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_prob(code, F, &ts))
        << "Bar CSV 喂完后应至少产出一次概率因子值";

    double sig = 0.0;
    ASSERT_TRUE(last_signal(code, sig))
        << "Bar CSV 喂完后应至少产出一次信号";

    // ========= 手工预先算好的期望值（基于当前 CSV 和上述 cfg） =========
    const double F_expected      = 0.4997473741056461;
    const double signal_expected = 0.0;

    // 容差给到 1e-8，避免不同平台上的极微小浮点误差
    EXPECT_NEAR(F, F_expected, 1e-8);
    EXPECT_DOUBLE_EQ(sig, signal_expected);
}

// ============== 用例 2：Transaction CSV 驱动（逐笔成交，带期望值对比） ==============
TEST(ProbabilityMomentum, TransactionsCsvFeed_ExpectationMatches) {
    BusGuard _guard;
    ProbabilityMomentumFactor::register_topics(4096);

    auto trans = testcfg::read_transactions_from_cfg();
    ASSERT_FALSE(trans.empty()) << "transactions_csv 未配置或无有效数据";
    ASSERT_GE(trans.size(), 40u) << "Transaction CSV 行数太少（请至少保证 >= 40 行）";

    const std::string code = trans[0].instrument_id;

    ProbMomentumConfig cfg;
    cfg.window_size      = 30;
    cfg.min_total_weight = 10.0;
    cfg.min_sigma        = 1e-6;
    cfg.buy_threshold    = 0.7;
    cfg.sell_threshold   = 0.3;
    cfg.debug_mode       = false;

    ProbabilityMomentumFactor factor({ code }, cfg);

    for (const auto& t : trans) {
        // IFactor::on_tick(const Transaction&) 会自动包装成 CombinedTick
        factor.on_tick(t);
    }

    double F = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_prob(code, F, &ts))
        << "Transaction CSV 喂完后应至少产出一次概率因子值";

    double sig = 0.0;
    ASSERT_TRUE(last_signal(code, sig))
        << "Transaction CSV 喂完后应至少产出一次信号";

    // ========= 手工预先算好的期望值（基于当前 CSV 和上述 cfg） =========
    const double F_expected      = 0.49340179477562607;
    const double signal_expected = 0.0;

    EXPECT_NEAR(F, F_expected, 1e-8);
    EXPECT_DOUBLE_EQ(sig, signal_expected);
}

// ============== 用例 3：Quote CSV 驱动（快照中间价，用 CSV 但只验合理性） ==============
TEST(ProbabilityMomentum, SnapshotCsvFeed_EmitsFiniteFactor) {
    BusGuard _guard;
    ProbabilityMomentumFactor::register_topics(4096);

    auto quotes = testcfg::read_quotes_from_cfg();
    ASSERT_FALSE(quotes.empty()) << "quotes_csv 未配置或无有效数据";
    ASSERT_GE(quotes.size(), 40u) << "Quote CSV 行数太少（请至少保证 >= 40 行）";

    const std::string code = quotes[0].instrument_id;

    ProbMomentumConfig cfg;
    cfg.window_size      = 30;
    cfg.min_total_weight = 10.0;
    cfg.min_sigma        = 1e-6;
    cfg.buy_threshold    = 0.7;
    cfg.sell_threshold   = 0.3;
    cfg.debug_mode       = false;

    ProbabilityMomentumFactor factor({ code }, cfg);

    for (const auto& q : quotes) {
        factor.on_quote(q);
    }

    double F = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_prob(code, F, &ts))
        << "Quote CSV 喂完后应至少产出一次概率因子值";

    EXPECT_TRUE(std::isfinite(F));
    EXPECT_GE(F, 0.0);
    EXPECT_LE(F, 1.0);

    double sig = 0.0;
    ASSERT_TRUE(last_signal(code, sig))
        << "Quote CSV 喂完后应至少产出一次信号";

    EXPECT_TRUE(std::isfinite(sig));
    EXPECT_GE(sig, -1.0);
    EXPECT_LE(sig,  1.0);
}
