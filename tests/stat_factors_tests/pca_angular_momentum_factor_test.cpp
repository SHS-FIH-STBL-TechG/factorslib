// tests/stat_factors_tests/pca_angular_momentum_factor_test.cpp

#include <gtest/gtest.h>
#include "../../src/stat_factors/pca_angular_momentum_factor.h"
#include "utils/databus.h"
#include "utils/types.h"

using namespace factorlib;

namespace {

struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

inline bool last_L(const std::string& code, double& v, int64_t* ts = nullptr) {
    auto& bus = DataBus::instance();
    return bus.get_latest<double>(TOP_PCA_ANG, code, v, ts);
}

} // namespace

// 思路：
//  - 构造一段“在 PC1-PC2 平面里绕圈”的人造轨迹：收益/成交量有周期性变化；
//  - 在线 PCA 会跟踪前两个主成分，特征在这两个主成分子空间里形成弯曲轨迹；
//  - 因子输出的 L_t = |u_{t-1} * v_t - v_{t-1} * u_t|
//    在这种“绕圈”情况下应当时不时明显大于 0，且始终为有限非负数。
TEST(PcaAngularMomentumFactor, ProducesFiniteAndNontrivialL) {
    BusGuard guard;
    PcaAngularMomentumFactor::register_topics(1024);

    const std::string code = "PCA2";

    PcaAngularMomentumConfig cfg;
    cfg.dims       = 3;
    cfg.k          = 2;      // 至少两个主成分
    cfg.lr         = 0.05;
    cfg.warmup     = 32;     // 预热期短一点，方便产出

    PcaAngularMomentumFactor factor({code}, cfg);

    double price = 100.0;
    const int N = 400;
    const int64_t base_ms = 1'000'000;
    const int64_t step_ms = 1'000; // 1s 一个 bar

    std::vector<double> L_vals;
    L_vals.reserve(N);

    for (int t = 0; t < N; ++t) {
        // 沿着“圆弧”移动的走势：ret 周期性变化
        double ret = 0.001 * std::sin(0.05 * t);
        price *= std::exp(ret);

        Bar b;
        b.instrument_id = code;
        b.data_time_ms  = base_ms + t * step_ms;
        b.open   = price * (1.0 - 0.0005);
        b.high   = price * (1.0 + 0.0010);
        b.low    = price * (1.0 - 0.0010);
        b.close  = price;
        b.volume = 100 + static_cast<uint64_t>(10 * std::cos(0.05 * t)); // 周期性体量
        b.turnover = b.close * static_cast<double>(b.volume);
        b.interval_ms = static_cast<int>(step_ms);

        factor.on_bar(b);

        double L = 0.0;
        int64_t ts = 0;
        if (last_L(code, L, &ts)) {
            ASSERT_TRUE(std::isfinite(L));
            ASSERT_GE(L, 0.0);
            L_vals.push_back(L);
        }
    }

    // 至少要有一些输出
    ASSERT_FALSE(L_vals.empty()) << "未从 DataBus 读到任何 PCA 角动量因子值";

    // 检查最大值是否明显大于 0，证明在绕圈轨迹下有非平凡弯曲
    double L_max = 0.0;
    for (double v : L_vals) {
        if (v > L_max) L_max = v;
    }

    EXPECT_GT(L_max, 1e-4) << "在非直线轨迹下，PCA 角动量 L 应出现明显大于 0 的值";
}
