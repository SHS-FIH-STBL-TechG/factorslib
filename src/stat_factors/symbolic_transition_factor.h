#pragma once
/**
 * @file symbolic_transition_factor.h
 * @brief 【时因子 #32】时间序列符号化转移矩阵因子（主特征值/谱半径近似）。
 *
 * 实现要点：
 *   - 使用 `math::RollingSymbolicDynamics` 将对数收益符号化（k 等分分箱）；
 *   - 在线维护一阶马尔可夫转移计数与概率；
 *   - 输出转移矩阵的谱半径（理论上 ≤ 1，主特征值 ≈ 1；越接近 1 说明模式越确定）。
 *
 * 参考：factor_design.md（时因子-符号化转移矩阵）fileciteturn0file0
 */

#include <string>
#include <unordered_map>
#include "ifactor.h"
#include "utils/types.h"
#include "utils/databus.h"
#include "utils/log.h"
#include "math/symbolic_dynamics.h"
#include "../config/runtime_config.h"

namespace factorlib {

struct SymbolicCfg {
    int    window_size = 256;
    int    symbols_k   = 6;
    bool   debug_mode  = false;
};

inline constexpr const char* TOP_SYMBOLIC_EIG1 = "time/symbolic_eig1";

class SymbolicTransitionFactor : public BaseFactor {
public:
    explicit SymbolicTransitionFactor(const std::vector<std::string>& codes,
                                      const SymbolicCfg& cfg = {});
    static void register_topics(size_t capacity=2048) {
        DataBus::instance().register_topic<double>(TOP_SYMBOLIC_EIG1, capacity);
    }

    void on_quote(const QuoteDepth& q) override;
    void on_tick (const CombinedTick& x) override;
    void on_bar  (const Bar& b) override;
    bool force_flush(const std::string& code) override { (void)code; return false; }

private:
    struct CodeState {
        bool has_last=false;
        double last_p=0.0;
        std::unique_ptr<math::RollingSymbolicDynamics<double>> sym;
    };
    SymbolicCfg _cfg;
    std::unordered_map<std::string, CodeState> _states;

    void ensure_state(const std::string& code);
    void on_price_event(const std::string& code, int64_t ts, double price);
    void maybe_publish(const std::string& code, int64_t ts);
};

} // namespace factorlib
