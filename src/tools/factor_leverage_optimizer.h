#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace factorlib::tools {

/**
 * @brief 网格搜索配置：定义 θ 范围、风险基准等。
 */
struct LeverageSearchConfig {
    double theta_min = 0.0;
    double theta_max = 2.5;
    double theta_step = 0.05;

    int D = 4000; // 0 表示自动使用样本长度               // 风险基准——默认 250 天满仓 1 倍
    double max_leverage = 2.0; // 全局杠杆上界

    double z_cap_quantile = 0.99; // 将 |z| 映射到 [1,2] 的最大分位
    double z_cap_min = 1.5;
    double z_cap_max = 6.0;

    double symmetry_score_threshold = 0.08;
    double discrete_unique_ratio_threshold = 0.15;
    double discrete_max_freq_ratio_threshold = 0.03;
};

enum class FactorDistributionKind {
    SymmetricContinuous,
    Asymmetric,
    DiscreteOrNonContinuous,
};

enum class TradeMode {
    BothSides,
    PositiveOnly,
    NegativeOnly,
};

struct FactorProfile {
    FactorDistributionKind kind = FactorDistributionKind::SymmetricContinuous;
    double symmetry_score = 0.0;
    double unique_ratio = 1.0;
    double max_freq_ratio = 0.0;
    double median = 0.0;
};

struct ThresholdSearchResult {
    bool ok = false;

    int D_sample_days = 0; // 分母与风险对齐使用的 D（样本天数）
    int T_trade_days = 0;  // 分子使用的 T（因子加入后交易天数）
    double theta = 0.0;
    double score = -1e100;
    // 基准（不使用因子，等价于 T=D 全样本 1x 持有）的 score。
    double baseline_score = std::numeric_limits<double>::quiet_NaN();
    double final_equity = 1.0;
    double dd_rms = 0.0;
    double c_scale = 1.0;
    double z_cap = 3.0;

    TradeMode mode = TradeMode::BothSides;
    int polarity = +1;
    std::size_t trade_days = 0;
    double theta_raw_low = std::numeric_limits<double>::quiet_NaN();
    double theta_raw_high = std::numeric_limits<double>::quiet_NaN();
    FactorProfile profile;
};

struct LeveragePoint {
    int64_t ts_ms = 0;
    double x_raw = 0.0;
    double z = 0.0;
    double b_raw = 0.0;
    double leverage = 0.0;
    double ret = 0.0;
    double equity = 1.0;
    double drawdown = 0.0;
    bool active = false;
};

class FactorLeverageOptimizer {
public:
    explicit FactorLeverageOptimizer(LeverageSearchConfig cfg);

    FactorProfile analyze_profile(const std::vector<double>& x) const;
    std::vector<double> rank_normalize_to_z(const std::vector<double>& x_centered) const;
    ThresholdSearchResult search_best_threshold(const std::vector<int64_t>& ts,
                                                const std::vector<double>& x_raw,
                                                const std::vector<double>& z,
                                                const std::vector<double>& next_ret,
                                                const std::vector<double>& full_next_ret,
                                                int D_sample_days) const;
    std::vector<LeveragePoint> build_leverage_series(const std::vector<int64_t>& ts,
                                                     const std::vector<double>& x_raw,
                                                     const std::vector<double>& z,
                                                     const std::vector<double>& next_ret,
                                                     int D_sample_days,
                                                     const ThresholdSearchResult& best) const;

private:
    LeverageSearchConfig _cfg;

    double finalize_leverage(double leverage, double raw_signal) const;
};

} // namespace factorlib::tools
