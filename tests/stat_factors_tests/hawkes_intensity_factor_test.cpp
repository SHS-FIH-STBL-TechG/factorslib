// tests/stat_factors_tests/hawkes_intensity_factor_test.cpp

#include "factors/stat/hawkes_intensity_factor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "core/databus.h"
#include "core/types.h"

using namespace factorlib;

// 为了测试，把 HawkesIntensityFactor 包一层，补齐 on_quote，使之非抽象
struct HawkesIntensityFactorForTest : public HawkesIntensityFactor {
    using HawkesIntensityFactor::HawkesIntensityFactor;

    void on_quote(const QuoteDepth& /*q*/) override {
        // 对于本测试，我们只关心 on_tick 的行为，这里留空即可
    }
};

namespace {

struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

bool last_hawkes(const std::string& code, double& out, int64_t* ts = nullptr) {
    return DataBus::instance().get_latest<double>(TOP_HAWKES, code, out, ts);
}

} // namespace

TEST(HawkesIntensityFactor, SyntheticTransactions_MatchDiscreteModel) {
    BusGuard guard;
    HawkesIntensityFactor::register_topics(2048);

    const std::string code = "TEST_HAWKES";

    // 选一组简单参数，便于计算
    HawkesCfg cfg;
    cfg.mu         = 0.1;
    cfg.alpha      = 0.3;
    cfg.beta       = 1.0;
    cfg.dt         = 1.0;    // Δt = 1s

    HawkesIntensityFactorForTest factor({code}, cfg);

    // 构造一段人造成交序列：固定时间间隔 Δt = 1s
    const int N = 20;
    const int64_t base_ms = 1'000'000;   // 任意起始时间
    const int64_t step_ms = static_cast<int64_t>(cfg.dt * 1000.0);

    std::vector<double> lambda_factor;   // 从因子读出的 λ_k
    lambda_factor.reserve(N);

    for (int k = 0; k < N; ++k) {
        Transaction tr;
        tr.instrument_id = code;
        tr.data_time_ms  = base_ms + k * step_ms;
        tr.main_seq      = static_cast<uint64_t>(k + 1);
        tr.price         = 1.0;
        tr.side          = 1;
        tr.volume        = 1;
        tr.bid_no        = 0;
        tr.ask_no        = 0;

        // IFactor 里已有 from Transaction 的 on_tick 重载
        factor.on_tick(tr);

        double lambda = 0.0;
        int64_t ts    = 0;
        ASSERT_TRUE(last_hawkes(code, lambda, &ts))
            << "第 " << k << " 个事件后未能从 DataBus 读到 Hawkes 强度";

        ASSERT_TRUE(std::isfinite(lambda));
        lambda_factor.push_back(lambda);
    }

    // ========= 在测试端按离散 Hawkes 模型重算 λ_k =========
    std::vector<double> lambda_expected;
    lambda_expected.reserve(N);

    const double mu    = cfg.mu;
    const double alpha = cfg.alpha;
    const double beta  = cfg.beta;
    const double dt    = cfg.dt;
    const double decay = std::exp(-beta * dt);

    double lambda_prev = mu;   // 设初始 λ_0 = μ

    for (int k = 0; k < N; ++k) {
        const double n_t = 1.0;  // 每步 1 个事件

        double lambda_cur = mu + decay * (lambda_prev - mu) + alpha * n_t;
        lambda_expected.push_back(lambda_cur);
        lambda_prev = lambda_cur;
    }

    ASSERT_EQ(lambda_expected.size(), lambda_factor.size());

    // ========= 对比两边序列 =========
    for (int k = 0; k < N; ++k) {
        double lf = lambda_factor[k];
        double le = lambda_expected[k];

        double diff = std::fabs(lf - le);
        double tol  = 1e-8 * std::max(1.0, std::fabs(le));

        EXPECT_LE(diff, tol) << "k=" << k
                             << " factor=" << lf
                             << " expected=" << le;
    }
}
