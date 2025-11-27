// tests/stat_factors_tests/memory_kernel_decay_factor_test.cpp

#include "factors/stat/memory_kernel_decay_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "core/databus.h"
#include "core/types.h"

using namespace factorlib;

namespace {

inline int64_t ms_of(int h, int m, int s, int ms = 0) {
    return (((static_cast<int64_t>(h) * 60 + m) * 60) + s) * 1000 + ms;
}

struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

bool last_mem_kernel(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_MEMK, code, out, ts);
}

std::vector<Bar> make_bars(const std::string& code, size_t count) {
    std::vector<Bar> bars;
    bars.reserve(count);
    double px = 50.0;
    int64_t base_ts = ms_of(10, 0, 0, 0);
    for (size_t i = 0; i < count; ++i) {
        double ret = 0.001 + 0.0007 * std::cos(0.2 * static_cast<double>(i));
        px *= std::exp(ret);
        Bar b{};
        b.instrument_id = code;
        b.data_time_ms  = base_ts + static_cast<int64_t>(i) * 60'000;
        b.close         = px;
        b.volume        = 500 + static_cast<int64_t>(i) * 15;
        bars.push_back(b);
    }
    return bars;
}

} // namespace

// 基本用例：检查合成 Bar 序列下能否得到有限的记忆核 M
TEST(MemoryKernelDecayFactor, SyntheticBarsProduceFiniteKernel) {
    BusGuard guard;
    MemoryKernelDecayFactor::register_topics(2048);

    const std::string code = "MEM_SYN";
    auto bars = make_bars(code, 180);

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

// 多配置用例：不同 (L, alpha) 组合仍能产出有限值
TEST(MemoryKernelDecayFactor, SyntheticBarsDifferentConfig) {
    BusGuard guard;
    MemoryKernelDecayFactor::register_topics(2048);

    const std::string code = "MEM_SYN_ALT";
    auto bars = make_bars(code, 200);

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
