#include <gtest/gtest.h>
#include "../../src/stat_factors/memory_kernel_decay_factor.h"
#include "utils/databus.h"
#include "utils/types.h"
#include <cmath>

using namespace factorlib;

namespace {
inline bool last_val(const std::string& code, double& v) {
    auto& bus = DataBus::instance();
    int64_t ts=0;
    return bus.get_latest<double>(TOP_MEM_KERNEL, code, v, &ts);
}
}

TEST(MemoryKernelDecayFactor, AR1PositiveMemory) {
    MemoryKernelDecayFactor::register_topics(1024);
    std::string code="MK1";
    MemKernelConfig cfg; cfg.window_size=128; cfg.L=20; cfg.alpha=0.6;
    MemoryKernelDecayFactor f({code}, cfg);
    // 生成 AR(1) 对数收益，并回放为价格
    double phi=0.8, eps=0.01;
    double p=100.0, ret=0.0;
    for (int t=0;t<300;++t) {
        ret = phi*ret + eps; p = p*std::exp(ret);
        QuoteDepth q; q.instrument_id=code; q.data_time_ms=t; q.bid_price=p-0.01; q.ask_price=p+0.01;
        f.on_quote(q);
    }
    double m=0.0; ASSERT_TRUE(last_val(code,m));
    ASSERT_GT(m, 0.0) << "正相关记忆应得到正的记忆核";
}
