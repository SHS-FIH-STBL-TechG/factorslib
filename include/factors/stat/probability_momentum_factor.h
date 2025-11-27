#pragma once
/**
 * @file probability_momentum_factor.h
 * @brief 概率动量因子：
 *        使用成交量为权重，对近期对数收益率做加权正态假设，
 *        计算下一步收益为正的概率 F，并给出买卖信号。
 */

#include <string>
#include <unordered_map>
#include <vector>

#include "core/ifactor.h"
#include "core/types.h"
#include "math/sliding_statistics.h"  // WeightedSlidingWindowStats

namespace factorlib {

// =====================[ 配置 ]=====================

struct ProbMomentumConfig {
    int    window_size      = 60;    ///< 窗口内样本个数（收益条数）
    double min_total_weight = 1.0;   ///< 计算概率所需的最小权重和
    double min_sigma        = 1e-6;  ///< 标准差下限，防止除零
    double buy_threshold    = 0.7;   ///< 买入信号阈值：F >= buy_threshold
    double sell_threshold   = 0.3;   ///< 卖出信号阈值：F <= sell_threshold
};

// =====================[ 主题名 ]=====================

inline constexpr const char* TOP_PROB_MOM_PROB   = "prob_mom/prob_up";
inline constexpr const char* TOP_PROB_MOM_SIGNAL = "prob_mom/signal";

// =====================[ 因子类 ]=====================

class ProbabilityMomentumFactor : public BaseFactor {
public:
    ProbabilityMomentumFactor(const std::vector<std::string>& codes,
                              const ProbMomentumConfig& cfg = {});

    /// 注册 DataBus topic
    static void register_topics(size_t capacity = 1024);

    // IFactor 接口：支持三种数据驱动
    void on_quote(const QuoteDepth& q) override;
    void on_tick (const CombinedTick& x) override;
    void on_bar  (const Bar& b) override;

    bool force_flush(const std::string& code) override {
        (void)code;
        return false;
    }

    void on_code_added(const std::string& code) override;

private:
    struct CodeState {
        bool   has_last_price = false;
        double last_price     = 0.0;
        int    window_size    = 0;   ///< 当前 scope 生效的窗口长度
        math::WeightedSlidingWindowStats<double, double> stats; ///< 加权滑动统计
    };

    ProbMomentumConfig _cfg;
    std::vector<int> _window_sizes; ///< 从配置解析出来的所有窗口集合
    std::unordered_map<std::string, CodeState> _states;

    CodeState& ensure_state(const ScopeKey& scope);

    /// 统一价格事件入口：任何来源（快照/逐笔/K 线）的价格都走这里
    void on_price_event(const std::string& code_raw,
                        int64_t ts_ms,
                        double price,
                        double volume);

    /// 计算上涨概率并发布
    void compute_and_publish(const std::string& code,
                             CodeState& S,
                             int64_t ts_ms);
};

} // namespace factorlib
