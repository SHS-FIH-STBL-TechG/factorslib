// tests/stat_factors_tests/volume_mgf_factor_test.cpp

#include "factors/stat/volume_mgf_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <deque>
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

bool last_mgf(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_VOL_MGF, code, out, ts);
}

std::vector<Bar> make_bars(const std::string& code, size_t count) {
    std::vector<Bar> bars;
    bars.reserve(count);
    double px = 60.0;
    int64_t base_ts = ms_of(9, 50, 0, 0);
    for (size_t i = 0; i < count; ++i) {
        double ret = 0.0005 * std::cos(0.12 * static_cast<double>(i));
        px *= std::exp(ret);
        Bar b{};
        b.instrument_id = code;
        b.data_time_ms  = base_ts + static_cast<int64_t>(i) * 30'000;
        b.close         = px;
        b.volume        = 500 + static_cast<int64_t>((i % 10) + 1) * 25;
        bars.push_back(b);
    }
    return bars;
}

} // namespace

// 数学验证用例：对合成成交量按定义重算 MGF 并与因子输出对比
TEST(VolumeMGFFactor, SyntheticBarsMatchMathematicalDefinition) {
    BusGuard guard;
    VolumeMGFFactor::register_topics(2048);

    const std::string code = "VOL_MGF_SYN";
    auto bars = make_bars(code, 120);

    VolumeMGFConfig cfg;
    cfg.window_size = 64;    // 用一个相对小的窗口，便于滑窗滚动
    cfg.t           = 0.01;  // 与 runtime_config.ini 中默认保持一致或近似

    VolumeMGFFactor factor({code}, cfg);

    // 测试端自己的滑窗 + Σexp(t·v)
    std::deque<double> win;
    double sum_exp = 0.0;

    for (const auto& b : bars) {
        if (b.instrument_id != code) continue;

        // 用 Bar.volume 作为 v_t
        double v = static_cast<double>(b.volume);

        // 测试端维护的滑窗逻辑：与因子设计说明完全一致
        double e = std::exp(cfg.t * v);
        sum_exp += e;
        win.push_back(v);
        if (static_cast<int>(win.size()) > cfg.window_size) {
            double v_old = win.front();
            win.pop_front();
            sum_exp -= std::exp(cfg.t * v_old);
        }

        factor.on_bar(b);
    }

    ASSERT_FALSE(win.empty()) << "窗口内没有有效成交量，无法测试 MGF";

    double mgf_expected = sum_exp / static_cast<double>(win.size());

    double mgf_factor = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(last_mgf(code, mgf_factor, &ts))
        << "Bars CSV 喂完后，应至少产出一次 volume/mgf";

    // 数值严格对比：允许极小的浮点误差
    ASSERT_TRUE(std::isfinite(mgf_factor));
    ASSERT_TRUE(std::isfinite(mgf_expected));

    double diff = std::fabs(mgf_factor - mgf_expected);
    double tol  = 1e-10 * std::max(1.0, std::fabs(mgf_expected));
    EXPECT_LE(diff, tol) << "MGF 与按定义重算结果不一致";
}
