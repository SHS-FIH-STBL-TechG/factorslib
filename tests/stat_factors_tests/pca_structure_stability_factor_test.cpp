// tests/stat_factors_tests/pca_structure_stability_factor_test.cpp

#include "../../src/stat_factors/pca_structure_stability_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {

struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

bool last_pca_stab(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_PCA_STAB, code, out, ts);
}

} // namespace

// 思路：
//  - 构造一段“结构基本不变”的 Bar 序列：收益、成交量、VWAP 都缓慢平稳变动；
//  - 在线 PCA 的第一主成分方向不会剧烈跳变，相邻主方向夹角 cos ≈ 1；
//  - 因子输出 S_t = |<u_t, u_{t-1}>| 应该落在 [0,1]，且在稳定阶段接近 1。
TEST(PcaStructureStabilityFactor, SyntheticBars_StabilityNearOne) {
    BusGuard guard;
    PcaStructureStabilityFactor::register_topics(2048);

    const std::string code = "TEST_PCA_STAB";

    PcaStructureStabilityConfig cfg;
    cfg.dims       = 3;
    cfg.k          = 1;
    cfg.lr         = 0.05;
    cfg.warmup     = 16;   // 让预热期短一点，方便在测试中看到足够多的输出

    PcaStructureStabilityFactor factor({code}, cfg);

    // 构造一段人造 K 线：价格小幅单调上行，成交量 / turnover 稍微抖动
    const int N = 128;
    const int64_t base_ms = 1'000'000;
    const int64_t step_ms = 1'000;   // 1s 一个 Bar

    double price = 100.0;
    uint64_t base_vol = 1000;

    std::vector<double> stab_vals;
    stab_vals.reserve(N);

    for (int i = 0; i < N; ++i) {
        Bar b;
        b.instrument_id = code;
        b.data_time_ms  = base_ms + i * step_ms;

        // 构造一个“平滑但略有波动”的价格路径
        double ret = 0.0005;                  // 约 0.05% 的常数收益
        price *= std::exp(ret);               // 累乘得到平滑上涨价格

        b.open   = price * (1.0 - 0.0002);
        b.high   = price * (1.0 + 0.0005);
        b.low    = price * (1.0 - 0.0005);
        b.close  = price;
        b.volume = base_vol + static_cast<uint64_t>(i % 50);  // 轻微抖动
        b.turnover = b.close * static_cast<double>(b.volume);
        b.interval_ms = static_cast<int>(step_ms);

        factor.on_bar(b);

        double s = 0.0;
        int64_t ts = 0;
        if (last_pca_stab(code, s, &ts)) {
            // 所有输出必须在 [0,1] 且有限
            EXPECT_TRUE(std::isfinite(s));
            EXPECT_GE(s, 0.0);
            EXPECT_LE(s, 1.0);
            stab_vals.push_back(s);
        }
    }

    // 预热期过后，应该能拿到足够多的稳定性输出
    ASSERT_FALSE(stab_vals.empty()) << "未能从 DataBus 读到任何 PCA 稳定性因子值";

    // 取最后若干个点，检查其平均值应明显接近 1（说明结构相邻变化很小）
    const int tail = 20;
    double sum_tail = 0.0;
    int cnt_tail = 0;
    for (int i = static_cast<int>(stab_vals.size()) - tail; i < static_cast<int>(stab_vals.size()); ++i) {
        if (i < 0) continue;
        sum_tail += stab_vals[i];
        ++cnt_tail;
    }

    ASSERT_GT(cnt_tail, 0);
    double avg_tail = sum_tail / static_cast<double>(cnt_tail);

    // 在平稳结构下，平均稳定性应非常接近 1，这里给一个相对宽松但有意义的下界
    EXPECT_GT(avg_tail, 0.95) << "平稳结构下的 PCA 稳定性平均值应接近 1";
}