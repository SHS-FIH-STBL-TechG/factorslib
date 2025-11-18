// tests/stat_factors_tests/least_squares_info_gain_factor_test.cpp

#include "../../src/stat_factors/least_squares_info_gain_factor.h"

#include <gtest/gtest.h>

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

// 取某个 code 的最新 IG 值
bool last_ig(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_LSIG_IG, code, out, ts);
}

} // namespace

// ============== 用例 1：Bar CSV 驱动 ==============
TEST(LeastSquaresInfoGain, BarsCsvFeed_EmitsFactor) {
    BusGuard _guard;
    LeastSquaresInfoGainFactor::register_topics(2048);

    // 从 test_config.ini 的 [global].bars_csv 读取 Bar 序列
    auto bars = testcfg::read_bars_from_cfg();
    ASSERT_FALSE(bars.empty()) << "bars_csv 未配置或无有效数据";
    ASSERT_GE(bars.size(), 40u) << "Bar CSV 行数太少（请至少保证 >= 40 行）";

    const std::string code = bars[0].instrument_id;

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

// ============== 用例 2：Transaction CSV 驱动 ==============
TEST(LeastSquaresInfoGain, TransactionsCsvFeed_EmitsFactor) {
    BusGuard _guard;
    LeastSquaresInfoGainFactor::register_topics(4096);

    // 从 [global].transactions_csv 读取逐笔成交
    auto trans = testcfg::read_transactions_from_cfg();
    ASSERT_FALSE(trans.empty()) << "transactions_csv 未配置或无有效数据";
    ASSERT_GE(trans.size(), 40u) << "Transaction CSV 行数太少（请至少保证 >= 40 行）";

    const std::string code = trans[0].instrument_id;

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

// ============== 用例 3：Quote CSV 驱动（中间价）===============
TEST(LeastSquaresInfoGain, SnapshotCsvFeed_EmitsFactor) {
    BusGuard _guard;
    LeastSquaresInfoGainFactor::register_topics(4096);

    // 从 [global].quotes_csv 读取快照
    auto quotes = testcfg::read_quotes_from_cfg();
    ASSERT_FALSE(quotes.empty()) << "quotes_csv 未配置或无有效数据";
    ASSERT_GE(quotes.size(), 40u) << "Quote CSV 行数太少（请至少保证 >= 40 行）";

    const std::string code = quotes[0].instrument_id;

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

// ============== 用例 4：三种 CSV 混合驱动 ==============
TEST(LeastSquaresInfoGain, MixedCsvFeed_MultiSources) {
    BusGuard _guard;
    LeastSquaresInfoGainFactor::register_topics(8192);

    auto bars   = testcfg::read_bars_from_cfg();
    auto trans  = testcfg::read_transactions_from_cfg();
    auto quotes = testcfg::read_quotes_from_cfg();

    ASSERT_FALSE(bars.empty())   << "bars_csv 未配置或无有效数据";
    ASSERT_FALSE(trans.empty())  << "transactions_csv 未配置或无有效数据";
    ASSERT_FALSE(quotes.empty()) << "quotes_csv 未配置或无有效数据";

    const std::string code = bars[0].instrument_id;

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
