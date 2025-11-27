// tests/granger_causality_factor_test.cpp
#include "factors/stat/granger_causality_factor.h"
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "core/databus.h"
#include "core/types.h"
#include "../utils/test_config.h"

using namespace factorlib;

namespace {

inline int64_t ms_of(int h, int m, int s, int ms) {
    return ((int64_t)h*3600 + (int64_t)m*60 + s) * 1000 + ms;
}

struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

// 读最新值的工具
inline bool last_strength(const std::string& code, double& out, int64_t* ts=nullptr) {
    return DataBus::instance().get_latest<double>(TOP_GRANGER_STRENGTH, code, out, ts);
}
inline bool last_pval(const std::string& code, double& out, int64_t* ts=nullptr) {
    return DataBus::instance().get_latest<double>(TOP_GRANGER_PVAL, code, out, ts);
}

// 送一笔委托（累计到 OFI）
inline void feed_order(GrangerCausalityFactor& f, const std::string& code,
                       int64_t ts, int side, uint64_t vol) {
    Entrust e{};
    e.instrument_id = code;
    e.data_time_ms  = ts;
    e.price = 0.0;
    e.side  = side;   // 买>0，卖<=0
    e.volume= vol;
    f.on_tick(e);
}

// 送一条报价（触发 mid 刷新 -> 产出 y_t）
inline void feed_quote(GrangerCausalityFactor& f, const std::string& code,
                       int64_t ts, double mid) {
    QuoteDepth q{};
    q.instrument_id = code;
    q.data_time_ms  = ts;
    q.bid_price = mid - 0.01;
    q.ask_price = mid + 0.01;
    f.on_quote(q);
}

} // namespace

// ============== 用例 1：注册/冒烟 ==============
TEST(GrangerCausality, RegisterAndSmoke) {
    BusGuard _guard;

    // 注册两个 topic
    GrangerCausalityFactor::register_topics(1024);

    // 构造（默认逐条事件）
    GrangerConfig cfg;
    cfg.window_size   = 64;
    cfg.p_lags        = 1;
    cfg.q_lags        = 1;
    cfg.min_effective = 8;
    cfg.publish_raw_p = true;  // 同时发布 p 值用于断言
    GrangerCausalityFactor factor({ "SH600000" }, cfg);

    // 未到有效样本前不应有输出
    double v;
    EXPECT_FALSE(last_strength("SH600000", v));
    EXPECT_FALSE(last_pval("SH600000", v));
}

// ============== 用例 2：H0 真（无因果）p 值应偏大 ==============
TEST(GrangerCausality, NoCausality_HighP) {
    BusGuard _guard;
    GrangerCausalityFactor::register_topics(4096);

    GrangerConfig cfg;
    cfg.window_size   = 200;
    cfg.p_lags        = 1;
    cfg.q_lags        = 1;
    cfg.min_effective = 20;
    cfg.publish_raw_p = true;
    cfg.use_neglog10  = false; // 直接断言 p 值
    GrangerCausalityFactor factor({ "X" }, cfg);

    const std::string code = "X";
    double mid = 100.0;
    double phi = 0.6;     // y 的自回归系数
    double y_prev = 0.0;

    // x 与 y 独立：x_t 取交替符号的常数序列（不影响 y）
    for (int t=0; t<240; ++t) {
        int64_t ts = ms_of(9, 30, 0, t);

        double x_t = (t%2==0) ? 100.0 : -100.0;   // 只影响 OFI
        if (x_t > 0) feed_order(factor, code, ts, +1, (uint64_t)std::llround(std::fabs(x_t)));
        else         feed_order(factor, code, ts, -1, (uint64_t)std::llround(std::fabs(x_t)));

        // y_t 只由 y_{t-1} 决定（无 x 影响）
        double y_t = phi * y_prev;   // 噪声置零以稳定可重复
        y_prev = y_t;

        mid += y_t;
        feed_quote(factor, code, ts, mid);
    }

    double p=1.0; int64_t ts=0;
    ASSERT_TRUE(last_pval(code, p, &ts));
    // 无因果：p 应不小（> 0.1 较稳妥；可以据你阈值偏好调整）
    EXPECT_GT(p, 0.10) << "H0 真时 p 值应较大";
}

// ============== 用例 3：H1 真（强因果）p 值应很小 ==============
TEST(GrangerCausality, StrongCausality_LowP) {
    BusGuard _guard;
    GrangerCausalityFactor::register_topics(4096);

    GrangerConfig cfg;
    cfg.window_size   = 200;
    cfg.p_lags        = 1;   // y_{t-1}
    cfg.q_lags        = 1;   // x_{t-1}
    cfg.min_effective = 20;
    cfg.publish_raw_p = true;
    cfg.use_neglog10  = false;
    GrangerCausalityFactor factor({ "Y" }, cfg);

    const std::string code = "Y";
    double mid = 100.0;
    double phi = 0.2;     // 自回归较弱
    double beta= 0.8;     // x 的影响很强
    double y_prev = 0.0;
    double x_prev = 0.0;

    for (int t=0; t<240; ++t) {
        int64_t ts = ms_of(9, 30, 0, t);

        // 生成外生 x_t（正弦波 → 连续可微，避免数值尖锐）
        double x_t = std::sin(0.1 * t) * 100.0;

        // 先把 x_t 累进 OFI（订单），再用 quote 产出 y_t
        if (x_t >= 0) feed_order(factor, code, ts, +1, (uint64_t)std::llround(std::fabs(x_t)));
        else          feed_order(factor, code, ts, -1, (uint64_t)std::llround(std::fabs(x_t)));

        // y_t = φ y_{t-1} + β x_{t-1}  （注意：用上一期 x_{t-1}，满足格兰杰滞后关系）
        double y_t = phi * y_prev + beta * x_prev;
        y_prev = y_t;
        x_prev = x_t;

        mid += y_t;
        feed_quote(factor, code, ts, mid);
    }

    double p=1.0; int64_t ts=0;
    ASSERT_TRUE(last_pval(code, p, &ts));
    EXPECT_LT(p, 1e-6) << "强因果情形下 p 值应非常小";
}

// ============== 用例 4：滑窗滚动后一致性 ==============
TEST(GrangerCausality, WindowRoll_Consistency) {
    BusGuard _guard;
    GrangerCausalityFactor::register_topics(4096);

    GrangerConfig cfg;
    cfg.window_size   = 80;   // 故意用较小窗，方便滚动
    cfg.p_lags        = 1;
    cfg.q_lags        = 1;
    cfg.min_effective = 20;
    cfg.publish_raw_p = false;
    cfg.use_neglog10  = true;   // 检查强度（-log10 p）
    GrangerCausalityFactor factor({ "Z" }, cfg);

    const std::string code = "Z";
    double mid = 100.0;
    double phi = 0.1, beta = 1.2;
    double y_prev = 0.0, x_prev = 0.0;

    // 先“热身”一段
    for (int t=0; t<100; ++t) {
        int64_t ts = ms_of(9, 30, 0, t);
        double x_t = (t%10<5 ? 100.0 : -100.0);
        if (x_t >= 0) feed_order(factor, code, ts, +1, (uint64_t)std::llround(std::fabs(x_t)));
        else          feed_order(factor, code, ts, -1, (uint64_t)std::llround(std::fabs(x_t)));
        double y_t = phi*y_prev + beta*x_prev;
        y_prev = y_t; x_prev = x_t;
        mid += y_t; feed_quote(factor, code, ts, mid);
    }

    // 采两次末端强度，要求“保持高位”，例如 > 5（对应 p < 1e-5）
    double s1=0, s2=0;
    ASSERT_TRUE(last_strength(code, s1));
    // 再多滚动一段
    for (int t=100; t<160; ++t) {
        int64_t ts = ms_of(9, 30, 0, t);
        double x_t = (t%8<4 ? 80.0 : -80.0);
        if (x_t >= 0) feed_order(factor, code, ts, +1, (uint64_t)std::llround(std::fabs(x_t)));
        else          feed_order(factor, code, ts, -1, (uint64_t)std::llround(std::fabs(x_t)));
        double y_t = phi*y_prev + beta*x_prev;
        y_prev = y_t; x_prev = x_t;
        mid += y_t; feed_quote(factor, code, ts, mid);
    }
    ASSERT_TRUE(last_strength(code, s2));
    EXPECT_GT(s1, 5.0);
    EXPECT_GT(s2, 5.0);
}

// 黄金值校验：与因子输出逐样比对（ARX(1,1)，逐条事件）
TEST(GrangerCausality, GoldenCheck_WithEigen) {
    DataBus::instance().reset();
    GrangerCausalityFactor::register_topics(4096);

    GrangerConfig cfg;
    cfg.window_size   = 120;
    cfg.p_lags        = 1;
    cfg.q_lags        = 1;
    cfg.min_effective = 30;
    cfg.use_neglog10  = false;   // 我们比对 p 原值
    cfg.publish_raw_p = true;

    const std::string code = "GOLD.EIGEN";
    GrangerCausalityFactor fac({code}, cfg);

    // —— 构造确定性序列（无随机性，便于复现）——
    const int T = 200;
    std::vector<double> xs(T), ys(T), mids(T);
    double mid = 100.0, y_prev=0.0, x_prev=0.0;
    for (int t=0; t<T; ++t) {
        double x = std::sin(0.15 * t) * 50.0;  // x_t
        double y = 0.25 * y_prev + 0.9 * x_prev; // y_t = φ y_{t-1} + β x_{t-1}
        x_prev = x; y_prev = y;
        xs[t] = x; ys[t] = y;
        mid += y; mids[t] = mid;
    }

    // —— 喂给因子（逐条事件）——
    for (int t=0; t<T; ++t) {
        // 用 x_t 累 OFI
        if (xs[t] >= 0) {
            Entrust e{};
            e.instrument_id = code;
            e.data_time_ms  = t*2;
            e.price         = 0.0;
            e.side          = +1;
            e.volume        = static_cast<decltype(e.volume)>(std::llround(std::fabs(xs[t])));
            fac.on_tick(e);
        } else {
            Entrust e{};
            e.instrument_id = code;
            e.data_time_ms  = t*2;
            e.price         = 0.0;
            e.side          = -1;
            e.volume        = static_cast<decltype(e.volume)>(std::llround(std::fabs(xs[t])));
            fac.on_tick(e);
        }
        // 用 mid 触发 y_t
        QuoteDepth q{}; q.instrument_id=code; q.data_time_ms=t*2+1;
        q.bid_price = mids[t] - 0.01; q.ask_price = mids[t] + 0.01;
        fac.on_quote(q);
    }

    // —— 从 DataBus 取因子输出的 p ——
    double p_factor=1.0; int64_t ts=0;
    ASSERT_TRUE(DataBus::instance().get_latest<double>(TOP_GRANGER_PVAL, code, p_factor, &ts));

    // —— 自己用 Eigen 重算黄金值 ——
    // 先构造窗口内的样本（注意：有效样本要 >= p+q+1）
    const int N = cfg.window_size;
    const int p = cfg.p_lags, q = cfg.q_lags;
    const int k = q;
    ASSERT_GE(T, N + std::max(p,q) + 1);

    // 取最后 N 条样本，组受限/非受限模型
    Eigen::MatrixXd Xr(N, 1+p);  // [1, y_{t-1}]
    Eigen::MatrixXd Xu(N, 1+p+q); // [1, y_{t-1}, x_{t-1}]
    Eigen::VectorXd Y(N);

    int start = T - N;  // 取末尾窗口
    for (int i=0; i<N; ++i) {
        int t = start + i;
        double y_t = ys[t];
        double y_lag = ys[t-1];
        double x_lag = xs[t-1];

        Y(i) = y_t;
        Xr(i,0) = 1.0; Xr(i,1) = y_lag;
        Xu(i,0) = 1.0; Xu(i,1) = y_lag; Xu(i,2) = x_lag;
    }

    auto solve_rss = [](const Eigen::MatrixXd& X, const Eigen::VectorXd& Y, double& rss)->bool{
        Eigen::MatrixXd XtX = X.transpose() * X;
        Eigen::VectorXd Xty = X.transpose() * Y;
        Eigen::LDLT<Eigen::MatrixXd> ldlt(XtX);
        if (ldlt.info()!=Eigen::Success || ldlt.isNegative()) return false;
        Eigen::VectorXd beta = ldlt.solve(Xty);
        if (ldlt.info()!=Eigen::Success) return false;
        Eigen::VectorXd e = Y - X*beta;
        rss = e.squaredNorm();
        return true;
    };

    double RSSr=0, RSSu=0;
    ASSERT_TRUE(solve_rss(Xr, Y, RSSr));
    ASSERT_TRUE(solve_rss(Xu, Y, RSSu));
    int df2 = N - (p + q + 1);
    ASSERT_GT(df2, 0);

    double num = (RSSr - RSSu) / (double)k;
    if (num < 0) num = 0;
    double den = RSSu / (double)df2;
    ASSERT_GT(den, 0);

    double F = num / den;
    double p_gold = math::fisher_f_sf<double>(F, k, df2);

    // —— 与因子输出比对 ——
    ASSERT_TRUE(std::isfinite(p_gold));
    EXPECT_NEAR(p_factor, p_gold, 1e-8);
}
