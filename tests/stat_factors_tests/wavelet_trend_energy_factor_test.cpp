#include <gtest/gtest.h>
#include "../../src/stat_factors/wavelet_trend_energy_factor.h"
#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {
inline bool last_val(const std::string& code, double& v) {
    auto& bus = DataBus::instance();
    int64_t ts=0;
    return bus.get_latest<double>(TOP_WAVE_TREND, code, v, &ts);
}
}

TEST(WaveletTrendEnergyFactor, MonotonicUp_HigherTrendEnergy) {
    WaveletTrendEnergyFactor::register_topics(1024);

    std::string code="S1";
    WaveTrendConfig cfg;
    cfg.window_size = 64; cfg.levels_J=6; cfg.trend_start_j=4; cfg.wavelet="db4";
    WaveletTrendEnergyFactor f({code}, cfg);

    // 单调上升中间价
    for (int i=0;i<100;++i) {
        QuoteDepth q; q.instrument_id=code; q.data_time_ms=i; q.bid_price=100+i*0.01; q.ask_price=q.bid_price+0.01;
        f.on_quote(q);
    }
    double val=0.0; ASSERT_TRUE(last_val(code,val));
    ASSERT_GE(val, 0.5) << "上升趋势应有较高低频能量占比";
}

TEST(WaveletTrendEnergyFactor, ConfigurableLevels) {
    WaveletTrendEnergyFactor::register_topics(1024);
    std::string code="S2";
    // J=4, 与 J=6 结果不同（只验证可生成）
    WaveTrendConfig cfg4; cfg4.window_size=64; cfg4.levels_J=4; cfg4.trend_start_j=3; cfg4.wavelet="db4";
    WaveletTrendEnergyFactor f4({code}, cfg4);
    for (int i=0;i<100;++i) {
        QuoteDepth q; q.instrument_id=code; q.data_time_ms=i; q.bid_price=100+std::sin(i*0.2); q.ask_price=q.bid_price+0.01;
        f4.on_quote(q);
    }
    double v4=0.0; ASSERT_TRUE(last_val(code,v4));

    std::string code2="S3";
    WaveTrendConfig cfg6; cfg6.window_size=64; cfg6.levels_J=6; cfg6.trend_start_j=4; cfg6.wavelet="db4";
    WaveletTrendEnergyFactor f6({code2}, cfg6);
    for (int i=0;i<100;++i) {
        QuoteDepth q; q.instrument_id=code2; q.data_time_ms=i; q.bid_price=100+std::sin(i*0.2); q.ask_price=q.bid_price+0.01;
        f6.on_quote(q);
    }
    double v6=0.0; ASSERT_TRUE(last_val(code2,v6));
}
