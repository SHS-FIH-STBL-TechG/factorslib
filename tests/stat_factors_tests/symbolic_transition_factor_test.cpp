#include <gtest/gtest.h>
#include "factors/stat/symbolic_transition_factor.h"
#include "core/databus.h"
#include "core/types.h"

using namespace factorlib;

namespace {
inline bool last_val(const std::string& code, double& v) {
    auto& bus = DataBus::instance(); int64_t ts=0;
    return bus.get_latest<double>(TOP_SYMBOLIC_EIG1, code, v, &ts);
}
}

TEST(SymbolicTransitionFactor, EigenvalueNearOne) {
    SymbolicTransitionFactor::register_topics(1024);
    std::string code="SYM1";
    SymbolicCfg cfg; cfg.window_size=128; cfg.symbols_k=5;
    SymbolicTransitionFactor f({code}, cfg);

    // 近确定性转移（交替涨跌）
    double p=100.0;
    for (int t=0;t<300;++t) {
        double delta = (t%2==0)? +0.01 : -0.009; p *= (1.0+delta);
        QuoteDepth q; q.instrument_id=code; q.data_time_ms=t; q.bid_price=p-0.01; q.ask_price=p+0.01;
        f.on_quote(q);
    }
    double e1=0.0; ASSERT_TRUE(last_val(code,e1));
    ASSERT_LE(e1, 1.05);
    ASSERT_GE(e1, 0.8);
}
