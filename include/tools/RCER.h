// FactorRCER.h
// 说明：
//   这个类封装了：
//   1. 用 K 线收盘价计算“满仓持有该股票”的基准 Score（收益 / 回撤效率）
//   2. 根据因子值和阈值，生成三值信号 S_t ∈ {-1, 0, 1} （顺便输出平滑置信度）
//   3. 在给定阈值下，计算该因子策略的 RCER（相对于满仓基准的效率比）
//   4. 在一组分位数网格上搜索最优阈值（使 RCER 最大）
//
// 使用方式（示例伪代码）:
//   #include "FactorRCER.h"
//   using namespace quant;
//
//   FactorRCER evaluator(250, 252.0); // 窗口=250, 一年252交易日
//   double baseScore = evaluator.computeBaseScore(prices); // 满仓基准 Score_0
//
//   // 例如用网格搜索找到最优阈值：
//   std::vector<double> lowQ = {0.1, 0.2, 0.3};
//   std::vector<double> highQ = {0.7, 0.8, 0.9};
//   double bestLowThr, bestHighThr, bestRCER;
//   evaluator.searchBestThresholdsByRCER(
//       prices, factor, lowQ, highQ,
//       baseScore,
//       bestLowThr, bestHighThr, bestRCER
//   );
//
//   // 如果只想用固定分位数：
//   double lowThr, highThr;
//   evaluator.findThresholdsQuantile(factor, 0.3, 0.7, lowThr, highThr);
//   double rcer = evaluator.calcRCERForThreshold(prices, factor, lowThr, highThr, baseScore);

#pragma once

#include <vector>

namespace factorlib {

class FactorRCER {
public:
    // 构造函数
    // 参数:
    //   window:    计算滚动最大回撤的窗口长度（单位：交易日，如 250 表示一年）
    //   tradingDaysPerYear: 年化换算使用的交易日个数（A 股一般 252）
    FactorRCER(int window = 250, double tradingDaysPerYear = 252.0);

    // 计算“满仓持有该股票”的基准 Score_0（收益 / 回撤效率）
    // 输入:
    //   prices: 收盘价序列 P_0..P_N，长度至少为 2
    // 返回:
    //   Score_0（基准策略的 Calmar 风格得分），如果数据无效则返回 0
    double computeBaseScore(const std::vector<double>& prices) const;

    // 使用固定分位数确定因子的低/高阈值
    // 输入:
    //   factor:      因子值序列 F_1..F_N，对应 N 个交易日（长度一般等于 prices.size() - 1）
    //   lowQuantile: 低分位数，例如 0.3
    //   highQuantile:高分位数，例如 0.7
    // 输出:
    //   outLowThr:   对应低分位数的因子阈值
    //   outHighThr:  对应高分位数的因子阈值
    void findThresholdsQuantile(const std::vector<double>& factor,
                                double lowQuantile,
                                double highQuantile,
                                double& outLowThr,
                                double& outHighThr) const;

    // 根据 lowThr, highThr 把因子转换为三值信号 S_t ∈ {-1,0,1}，并输出一个 [0,1] 的置信度
    // 规则:
    //   F_t >= highThr -> S_t = +1 （做多）
    //   F_t <= lowThr  -> S_t = -1 （做空）
    //   中间           -> S_t = 0  （空仓）
    //
    // 置信度 conf_t:
    //   在做多区间 [highThr, maxF] 内，越靠近 maxF 置信度越高
    //   在做空区间 [minF,  lowThr] 内，越靠近 minF  置信度越高
    //   中间区域       -> conf_t = 0
    void buildSignalsAndConfidence(const std::vector<double>& factor,
                                   double lowThr,
                                   double highThr,
                                   std::vector<int>& outSignals,
                                   std::vector<double>& outConf) const;

    // 使用给定阈值计算该因子策略的 RCER（相对满仓基准的 Score 比例）
    // 输入:
    //   prices:   收盘价 P_0..P_N
    //   factor:   因子值 F_1..F_N，长度应为 N（即 prices.size() - 1）
    //   lowThr:   做空阈值
    //   highThr:  做多阈值
    //   baseScore:满仓基准 Score_0（可以通过 computeBaseScore 得到）
    // 返回:
    //   RCER = Score_strat / Score_base
    double calcRCERForThreshold(const std::vector<double>& prices,
                                const std::vector<double>& factor,
                                double lowThr,
                                double highThr,
                                double baseScore) const;

    // 在一组候选分位数网格上搜索最优阈值，使 RCER 最大
    // 输入:
    //   prices, factor: 同上
    //   lowQuantiles:   一组低分位数候选，例如 {0.1, 0.2, 0.3}
    //   highQuantiles:  一组高分位数候选，例如 {0.7, 0.8, 0.9}
    //   baseScore:      满仓基准 Score_0
    // 输出:
    //   outBestLowThr:  最优低阈值
    //   outBestHighThr: 最优高阈值
    //   outBestRCER:    对应的最大 RCER
    void searchBestThresholdsByRCER(const std::vector<double>& prices,
                                    const std::vector<double>& factor,
                                    const std::vector<double>& lowQuantiles,
                                    const std::vector<double>& highQuantiles,
                                    double baseScore,
                                    double& outBestLowThr,
                                    double& outBestHighThr,
                                    double& outBestRCER) const;

private:
    int window_;                 // 滚动 MDD 的窗口长度
    double tradingDaysPerYear_;  // 一年多少交易日，用于年化

    // ====== 内部工具函数：实现细节，外部不需要直接调用 ======

    // 计算对数日收益 r_t = ln(P_t / P_{t-1})
    std::vector<double> calcLogReturns(const std::vector<double>& prices) const;

    // 根据对数收益序列构造净值曲线
    // 公式: V_{t+1} = V_t * exp(logReturns[t])
    std::vector<double> calcEquityFromLogReturns(const std::vector<double>& logReturns,
                                                 double v0 = 1.0) const;

    // 从净值曲线计算几何年化收益 CAGR
    double calcCAGR(const std::vector<double>& equity) const;

    // 计算净值曲线的平均滚动最大回撤（窗口长度使用 window_）
    double calcAvgRollingMDD(const std::vector<double>& equity) const;

    // Score = CAGR / AvgRollingMDD
    double calcScore(const std::vector<double>& equity) const;

    // 根据分位数计算因子的分位值
    double calcFactorQuantile(const std::vector<double>& factor,
                              double quantile) const;

    // 根据信号 S_t ∈ {-1,0,1} 和标的对数收益，构造策略对数收益
    // 公式: R_strat,t = L * S_t * r_m,t
    // 其中 L = N / N_active
    std::vector<double> buildStrategyLogReturnsWithLeverage(const std::vector<double>& logReturns,
                                                            const std::vector<int>& signals,
                                                            double& outLeverage) const;
};

} // namespace quant
