#include <gtest/gtest.h>
#include <vector>

#include "tools/factor_ic_runtime.h"
#include "factors/stat/memory_kernel_decay_factor.h"
#include "core/databus.h"
#include "core/types.h"

using namespace factorlib;

#if FACTORLIB_ENABLE_IC_RUNTIME

namespace {

Bar make_bar(const std::string& code, int64_t ts_ms, double close) {
    Bar b;
    b.instrument_id = code;
    b.data_time_ms = ts_ms;
    b.close = close;
    b.open = b.high = b.low = close;
    b.interval_ms = 60'000;
    return b;
}

} // namespace

// 验证：在 DataBus 上提前写入记忆核结果后，runtime 能够正确生成样本并保留表名。
TEST(FactorIcRuntimeTest, CollectsSamplesFromBars) {
    DataBus::instance().reset();
    MemoryKernelDecayFactor::register_topics(128);
    tools::ic_runtime_clear();

    const std::string code = "IC_TEST";
    std::vector<Bar> bars;
    for (int i = 0; i < 4; ++i) {
        bars.push_back(make_bar(code, 1'000 + i * 1'000, 100.0 + i));
    }

    // DataBus 上写入三条记忆核输出（对应前三根 Bar 的时间戳）
    for (int i = 0; i < 3; ++i) {
        double value = 0.1 * (i + 1);
        DataBus::instance().publish<double>(TOP_MEMK, code, bars[i].data_time_ms, value);
    }

    tools::ic_runtime_ingest_bars(bars, "test_table.csv");
    auto reports = tools::ic_runtime_snapshot_reports();
    ASSERT_EQ(reports.size(), 1);
    const auto& report = reports.front();
    EXPECT_EQ(report.code, code);
    EXPECT_EQ(report.table_name, "test_table.csv");
    ASSERT_EQ(report.topics.size(), 1);
    const auto& series = report.topics.front();
    EXPECT_EQ(series.topic, TOP_MEMK);
    ASSERT_EQ(series.samples.size(), 3);
    EXPECT_NEAR(series.samples.front().factor, 0.1, 1e-12);
    EXPECT_NEAR(series.samples.front().forward, bars[1].close / bars[0].close - 1.0, 1e-12);
    EXPECT_TRUE(series.latest_factor.has_value());

    tools::ic_runtime_clear();
    EXPECT_TRUE(tools::ic_runtime_snapshot_reports().empty());
}

// 验证：若 DataBus 中没有对应时间戳的因子值，则不会生成样本。
TEST(FactorIcRuntimeTest, MissingFactorValueProducesNoSamples) {
    DataBus::instance().reset();
    MemoryKernelDecayFactor::register_topics(128);
    tools::ic_runtime_clear();

    std::vector<Bar> bars = {
        make_bar("IC_EMPTY", 10'000, 50.0),
        make_bar("IC_EMPTY", 20'000, 55.0)
    };

    tools::ic_runtime_ingest_bars(bars, "empty.csv");
    auto reports = tools::ic_runtime_snapshot_reports();
    EXPECT_TRUE(reports.empty());
}

#else

TEST(FactorIcRuntimeTest, DisabledMacroProducesNoop) {
    tools::ic_runtime_clear();
    EXPECT_FALSE(tools::ic_runtime_enabled());
    EXPECT_TRUE(tools::ic_runtime_snapshot_reports().empty());
}

#endif
