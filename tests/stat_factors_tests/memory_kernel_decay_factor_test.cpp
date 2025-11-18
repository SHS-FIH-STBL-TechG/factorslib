// tests/stat_factors_tests/memory_kernel_decay_factor_test.cpp

#include "../../src/stat_factors/memory_kernel_decay_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "utils/databus.h"
#include "utils/types.h"
#include "../utils/test_config.h"   // testcfg::read_bars_from_cfg()

using namespace factorlib;

namespace {

struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

bool last_mem_kernel(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_MEMK, code, out, ts);
}

} // namespace

// 基本用例：检查能否从 Bars CSV 回放中得到有限的记忆核 M
TEST(MemoryKernelDecayFactor, BarsCsvFeed_Basic) {
    BusGuard guard;
    MemoryKernelDecayFactor::register_topics(2048);

    auto bars = testcfg::read_bars_from_cfg();
    ASSERT_FALSE(bars.empty()) << "bars_csv 未配置或无有效数据";

    const std::string code = bars[0].instrument_id;

    MemoryKernelConfig cfg;
    cfg.window_size = 128;
    cfg.L           = 50;
    cfg.alpha       = 0.6;

    MemoryKernelDecayFactor factor({code}, cfg);

    for (const auto& b : bars) {
        if (b.instrument_id != code) continue;
        factor.on_bar(b);
    }

    double M = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_mem_kernel(code, M, &ts))
        << "未能从 DataBus 读到 memory_kernel/decay";

    EXPECT_TRUE(std::isfinite(M));
}

// 多配置用例：改变 L 和 alpha，至少保证不会崩，并能产出有限值
TEST(MemoryKernelDecayFactor, BarsCsvFeed_DifferentConfig) {
    BusGuard guard;
    MemoryKernelDecayFactor::register_topics(2048);

    auto bars = testcfg::read_bars_from_cfg();
    ASSERT_FALSE(bars.empty()) << "bars_csv 未配置或无有效数据";

    const std::string code = bars[0].instrument_id;

    MemoryKernelConfig cfg;
    cfg.window_size = 128;
    cfg.L           = 20;   // 更短的记忆阶数
    cfg.alpha       = 0.8;  // 更强的长记忆权重

    MemoryKernelDecayFactor factor({code}, cfg);

    for (const auto& b : bars) {
        if (b.instrument_id != code) continue;
        factor.on_bar(b);
    }

    double M = 0.0;
    ASSERT_TRUE(last_mem_kernel(code, M, nullptr));
    EXPECT_TRUE(std::isfinite(M));
}
