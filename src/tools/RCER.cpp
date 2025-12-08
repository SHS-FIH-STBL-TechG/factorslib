// FactorRCER.cpp
// 具体实现，配合 FactorRCER.h 使用

#include "tools/RCER.h"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

namespace factorlib {

// ==================== 构造函数 ====================

FactorRCER::FactorRCER(int window, double tradingDaysPerYear)
    : window_(window), tradingDaysPerYear_(tradingDaysPerYear) {}

// ==================== 公共接口实现 ====================

// 计算“满仓持有该股票”的基准 Score_0
double FactorRCER::computeBaseScore(const std::vector<double>& prices) const {
    // prices: P_0..P_N，长度必须 >= 2
    if (prices.size() <= 1) {
        return 0.0;
    }

    // 1. 计算标的的对数收益 r_t = ln(P_t / P_{t-1})
    std::vector<double> logRets = calcLogReturns(prices);

    // 2. 根据对数收益构造净值曲线（满仓持有）
    std::vector<double> equity = calcEquityFromLogReturns(logRets, 1.0);

    // 3. 用净值曲线计算 Score_0 = CAGR / AvgRollingMDD
    return calcScore(equity);
}

// 使用固定分位数确定因子的低/高阈值
void FactorRCER::findThresholdsQuantile(const std::vector<double>& factor,
                                        double lowQuantile,
                                        double highQuantile,
                                        double& outLowThr,
                                        double& outHighThr) const {
    outLowThr  = calcFactorQuantile(factor, lowQuantile);
    outHighThr = calcFactorQuantile(factor, highQuantile);
}

// 根据因子阈值生成三值信号与置信度
void FactorRCER::buildSignalsAndConfidence(const std::vector<double>& factor,
                                           double lowThr,
                                           double highThr,
                                           std::vector<int>& outSignals,
                                           std::vector<double>& outConf) const {
    size_t N = factor.size();
    outSignals.assign(N, 0);
    outConf.assign(N, 0.0);

    if (N == 0) return;

    // 因子整体的最小值与最大值，用于构造平滑置信度
    double minF = *std::min_element(factor.begin(), factor.end());
    double maxF = *std::max_element(factor.begin(), factor.end());

    // 防止阈值过于接近导致除 0
    double denomLong  = std::max(1e-12, maxF - highThr);
    double denomShort = std::max(1e-12, lowThr - minF);

    for (size_t i = 0; i < N; ++i) {
        double f = factor[i];

        if (f >= highThr) {
            // 做多区间
            outSignals[i] = +1;
            double conf = (f - highThr) / denomLong; // 越靠近 maxF 越接近 1
            if (conf < 0.0) conf = 0.0;
            if (conf > 1.0) conf = 1.0;
            outConf[i] = conf;
        } else if (f <= lowThr) {
            // 做空区间
            outSignals[i] = -1;
            double conf = (lowThr - f) / denomShort; // 越靠近 minF 越接近 1
            if (conf < 0.0) conf = 0.0;
            if (conf > 1.0) conf = 1.0;
            outConf[i] = conf;
        } else {
            // 中性区间：不交易
            outSignals[i] = 0;
            outConf[i] = 0.0;
        }
    }
}

// 使用给定阈值计算因子策略的 RCER = Score_strat / Score_base
double FactorRCER::calcRCERForThreshold(const std::vector<double>& prices,
                                        const std::vector<double>& factor,
                                        double lowThr,
                                        double highThr,
                                        double baseScore) const {
    // prices: P_0..P_N
    // factor: F_1..F_N （长度应 = N）
    if (prices.size() <= 1) {
        return 0.0;
    }
    std::size_t N_price = prices.size();
    std::size_t N_factor = factor.size();

    if (N_factor != N_price - 1) {
        // 因子长度应与对数收益长度一致，即 N
        return 0.0;
    }

    // 1. 计算标的对数收益序列（长度 N）
    std::vector<double> logRets = calcLogReturns(prices);

    // 2. 由阈值生成三值信号与置信度（置信度暂不用于仓位，只是附加信息）
    std::vector<int> signals;
    std::vector<double> conf;
    buildSignalsAndConfidence(factor, lowThr, highThr, signals, conf);

    // 3. 构造策略的对数收益 R_strat,t = L * S_t * r_m,t
    double leverage = 0.0;
    std::vector<double> stratLogRets = buildStrategyLogReturnsWithLeverage(logRets,
                                                                           signals,
                                                                           leverage);
    if (leverage == 0.0) {
        // 因子完全不交易，RCER 视为 0
        return 0.0;
    }

    // 4. 根据策略对数收益构造净值曲线
    std::vector<double> equityStrat = calcEquityFromLogReturns(stratLogRets, 1.0);

    // 5. 计算策略 Score_strat
    double scoreStrat = calcScore(equityStrat);

    if (baseScore <= 0.0) {
        // 如果基准 Score_0 无效或 <= 0，则 RCER 无法定义，这里返回 0
        return 0.0;
    }

    // 6. RCER = Score_strat / Score_base
    return scoreStrat / baseScore;
}

// 在一组分位数网格上搜索最优阈值，使 RCER 最大
void FactorRCER::searchBestThresholdsByRCER(const std::vector<double>& prices,
                                            const std::vector<double>& factor,
                                            const std::vector<double>& lowQuantiles,
                                            const std::vector<double>& highQuantiles,
                                            double baseScore,
                                            double& outBestLowThr,
                                            double& outBestHighThr,
                                            double& outBestRCER) const {
    outBestLowThr  = 0.0;
    outBestHighThr = 0.0;
    outBestRCER    = -std::numeric_limits<double>::infinity();

    // 简单实现：直接双重循环遍历所有 (lowQ, highQ) 组合
    for (double lowQ : lowQuantiles) {
        for (double highQ : highQuantiles) {
            if (lowQ >= highQ) {
                // 要求低分位 < 高分位
                continue;
            }

            double lowThr  = calcFactorQuantile(factor, lowQ);
            double highThr = calcFactorQuantile(factor, highQ);

            // 避免阈值过于接近
            if (std::fabs(highThr - lowThr) < 1e-12) {
                continue;
            }

            double rcer = calcRCERForThreshold(prices, factor, lowThr, highThr, baseScore);
            if (rcer > outBestRCER) {
                outBestRCER   = rcer;
                outBestLowThr = lowThr;
                outBestHighThr = highThr;
            }
        }
    }

    // 如果没有找到比 -inf 更好的 rcer，则说明搜索失败，此时 outBestRCER 仍为 -inf
    // 调用方可以据此判断是否需要做特殊处理
}

// ==================== 私有工具函数实现 ====================

// 计算对数收益 r_t = ln(P_t / P_{t-1})
std::vector<double> FactorRCER::calcLogReturns(const std::vector<double>& prices) const {
    std::vector<double> logRets;
    if (prices.size() <= 1) return logRets;
    logRets.reserve(prices.size() - 1);
    for (std::size_t i = 1; i < prices.size(); ++i) {
        double prev = prices[i - 1];
        double curr = prices[i];
        if (prev > 0.0 && curr > 0.0) {
            logRets.push_back(std::log(curr / prev));
        } else {
            logRets.push_back(0.0);
        }
    }
    return logRets;
}

// 根据对数收益序列构造净值曲线
// 公式: V_{t+1} = V_t * exp(logReturns[t])
std::vector<double> FactorRCER::calcEquityFromLogReturns(const std::vector<double>& logReturns,
                                                         double v0) const {
    std::vector<double> equity;
    equity.reserve(logReturns.size() + 1);
    double value = v0;
    equity.push_back(value);
    for (double r : logReturns) {
        value *= std::exp(r);
        equity.push_back(value);
    }
    return equity;
}

// 从净值曲线计算几何年化收益 CAGR
double FactorRCER::calcCAGR(const std::vector<double>& equity) const {
    if (equity.size() <= 1) return 0.0;
    double start = equity.front();
    double end = equity.back();
    if (!(start > 0.0) || !(end > 0.0)) return 0.0;
    double steps = static_cast<double>(equity.size() - 1);
    if (!(steps > 0.0)) return 0.0;
    double years = steps / tradingDaysPerYear_;
    if (!(years > 0.0)) return 0.0;
    double ratio = end / start;
    return std::pow(ratio, 1.0 / years) - 1.0;
}

// 计算净值曲线的平均滚动最大回撤（窗口长度使用 window_）
double FactorRCER::calcAvgRollingMDD(const std::vector<double>& equity) const {
    if (equity.size() <= 1 || window_ <= 0) return 0.0;
    const int n = static_cast<int>(equity.size());
    double sum_mdd = 0.0;
    int count = 0;
    for (int end = 0; end < n; ++end) {
        int start = std::max(0, end - window_);
        double peak = equity[start];
        double worst = 0.0;
        for (int idx = start; idx <= end; ++idx) {
            peak = std::max(peak, equity[idx]);
            if (!(peak > 0.0)) continue;
            double drawdown = (peak - equity[idx]) / peak;
            if (drawdown > worst) {
                worst = drawdown;
            }
        }
        sum_mdd += worst;
        ++count;
    }
    if (count == 0) return 0.0;
    return sum_mdd / static_cast<double>(count);
}

// Score = CAGR / AvgRollingMDD
double FactorRCER::calcScore(const std::vector<double>& equity) const {
    double cagr = calcCAGR(equity);
    double avgMdd = calcAvgRollingMDD(equity);
    if (avgMdd <= 0.0) return 0.0;
    return cagr / avgMdd;
}

// 根据分位数计算因子的分位值
double FactorRCER::calcFactorQuantile(const std::vector<double>& factor,
                                      double quantile) const {
    if (factor.empty()) return 0.0;
    std::vector<double> values;
    values.reserve(factor.size());
    for (double v : factor) {
        if (std::isfinite(v)) {
            values.push_back(v);
        }
    }
    if (values.empty()) return 0.0;
    double q = std::min(1.0, std::max(0.0, quantile));
    std::size_t idx = static_cast<std::size_t>(q * static_cast<double>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + idx, values.end());
    return values[idx];
}

// 根据信号 S_t ∈ {-1,0,1} 和标的对数收益，构造策略对数收益
// 公式: R_strat,t = L * S_t * r_m,t
// 其中 L = N / N_active
std::vector<double> FactorRCER::buildStrategyLogReturnsWithLeverage(const std::vector<double>& logReturns,
                                                                    const std::vector<int>& signals,
                                                                    double& outLeverage) const {
    std::vector<double> strat(logReturns.size(), 0.0);
    if (logReturns.size() != signals.size() || logReturns.empty()) {
        outLeverage = 0.0;
        return strat;
    }
    std::size_t active = 0;
    for (int s : signals) {
        if (s != 0) ++active;
    }
    if (active == 0) {
        outLeverage = 0.0;
        return strat;
    }
    double leverage = static_cast<double>(logReturns.size()) / static_cast<double>(active);
    outLeverage = leverage;
    for (std::size_t i = 0; i < logReturns.size(); ++i) {
        strat[i] = leverage * static_cast<double>(signals[i]) * logReturns[i];
    }
    return strat;
}

} // namespace factorlib
