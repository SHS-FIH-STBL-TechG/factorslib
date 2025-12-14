#pragma once
/**
 * @file bma_ensemble_drift_factor.h
 * @brief Bayesian Model Averaging（BMA）后验集成漂移因子。
 *
 * 理论依据：
 *   - 把多个趋势漂移估计器的结果用贝叶斯证据加权
 *   - 输出集成漂移：F_BMA = Σ w_m μ_m
 *
 * 使用 math::RollingBMAEnsemble 进行计算。
 */

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/databus.h"
#include "core/ifactor.h"
#include "core/types.h"
#include "math/rolling_bma_ensemble.h"

namespace factorlib {

/**
 * @brief BMA 集成漂移因子配置。
 */
struct BmaEnsembleDriftConfig {
    int window_size = 40;       ///< 滚动窗口长度
    double ema_alpha = 0.1;     ///< EMA 衰减因子
};

/**
 * @brief BMA 后验集成漂移因子。
 *
 * 使用 math::RollingBMAEnsemble 进行计算。
 */
class BmaEnsembleDriftFactor : public BaseFactor {
public:
    using Code = std::string;

    BmaEnsembleDriftFactor(
        const std::vector<Code>& codes,
        const BmaEnsembleDriftConfig& cfg = BmaEnsembleDriftConfig{}
    );

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;
    void on_quote(const QuoteDepth&) override {}
    void on_tick(const CombinedTick&) override {}

    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    struct CodeState {
        explicit CodeState(const BmaEnsembleDriftConfig& cfg);

        bool has_last_close;
        double last_close;
        math::RollingBMAEnsemble<double> bma;

        void push_bar(const Bar& b);
        bool ready() const { return bma.ready(); }
    };

    BmaEnsembleDriftConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
