#pragma once
/**
 * @file hawkes_intensity_factor.h
 * @brief 【时因子 #11】霍克斯过程激发强度因子。
 *
 * 定义：
 *   λ_{t+1} = μ + e^{-βΔt} (λ_t - μ) + α * n_t（离散时间近似）
 *   其中 n_t 为当前步事件数（可取成交事件=1），用于刻画“自激发”活跃度。
 *
 * 实现：
 *   - 使用 `math::HawkesIntensity`，每次事件调用 update(1.0)；
 *   - 可配置参数：[hawkes.mu / hawkes.alpha / hawkes.beta / hawkes.dt]。
 *
 * 参考：factor_design.md（时因子-霍克斯强度）fileciteturn0file0
 */

#include <string>
#include <unordered_map>
#include <vector>
#include "core/ifactor.h"
#include "core/types.h"
#include "core/databus.h"
#include "utils/log.h"
#include "math/hawkes_intensity.h"
#include "config/runtime_config.h"

namespace factorlib {

struct HawkesCfg {
    double mu    = 0.1;
    double alpha = 0.3;
    double beta  = 1.2;
    double dt    = 1.0;
};

inline constexpr const char* TOP_HAWKES = "time/hawkes_intensity";

class HawkesIntensityFactor : public BaseFactor {
public:
    explicit HawkesIntensityFactor(const std::vector<std::string>& codes,
                                   const HawkesCfg& cfg = {});
    static void register_topics(size_t capacity=2048) {
        DataBus::instance().register_topic<double>(TOP_HAWKES, capacity);
    }

    void on_tick(const CombinedTick& x) override;
    bool force_flush(const std::string& code) override { (void)code; return false; }

private:
    struct CodeState {
        math::HawkesIntensity hawkes{0.1,0.3,1.2,1.0};
        bool inited=false;
        void init(const HawkesCfg& c) { hawkes = math::HawkesIntensity(c.mu, c.alpha, c.beta, c.dt); inited=true; }
    };
    HawkesCfg _cfg;
    std::vector<int> _window_sizes; //< Hawkes 没有窗口概念，这里占位以复用 for_each_scope
    std::unordered_map<std::string, CodeState> _states;

    CodeState& ensure_state(const ScopeKey& scope);
};

} // namespace factorlib
