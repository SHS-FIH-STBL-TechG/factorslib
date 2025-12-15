#pragma once
/**
 * @file wavelet_trend_energy_factor.h
 * @brief 小波趋势能量主导因子（价因子 #3）。
 *
 * 思路：
 *   - 使用 RollingMODWT<double> 对价格序列做最大重叠小波分解；
 *   - 对每个尺度 j 维护 detail 系数能量 E_j；
 *   - 趋势能量比 = (j >= trend_start_j 的能量之和) / (全部能量之和);
 *   - 通过 DataBus 发布“带方向”的趋势强度：sign(ΔlogP) * ratio，范围 [-1,1]。
 *
 * 实现约束：
 *   - 只依赖 include/math/modwt.h 中真实存在的接口：
 *       - RollingMODWT<double>(W, J, WaveletFilter)
 *       - push(double), ready(), trend_energy_ratio(int)
 *   - 只使用 Bar / QuoteDepth / CombinedTick 中的真实字段：
 *       - Bar::instrument_id / data_time_ms / close
 *       - QuoteDepth::instrument_id / data_time_ms / bid_price / ask_price
 *       - CombinedTick::instrument_id / data_time_ms / price
 */

#include <string>
#include <deque>
#include <unordered_map>
#include <vector>

#include "core/ifactor.h"
#include "core/types.h"
#include "math/modwt.h"

namespace factorlib {

// =====================[ 配置 ]=====================

struct WaveTrendConfig {
    int         window_size   = 128;     ///< MODWT 滑窗长度
    int         levels_J      = 6;       ///< 小波分解层数 J
    int         trend_start_j = 4;       ///< 视为“趋势”的起始尺度 j
    std::string wavelet       = "db4";   ///< "db4" 或 "sym4"
};

// 因子输出的 topic 名（单测和业务都会用到）
inline constexpr const char* TOP_WAVE_TREND = "wave_trend/energy_ratio";

// =====================[ 因子类 ]=====================

class WaveletTrendEnergyFactor : public BaseFactor {
public:
    WaveletTrendEnergyFactor(const std::vector<std::string>& codes,
                             const WaveTrendConfig& cfg = {});

    /// 注册 DataBus topic
    static void register_topics(std::size_t capacity = 1024);

    // IFactor 接口
    void on_quote(const QuoteDepth& q) override;
    void on_tick (const CombinedTick& x) override;
    void on_bar  (const Bar& b) override;

    bool force_flush(const std::string& /*code*/) override {
        return false;
    }

    void on_code_added(const std::string& code) override;

private:
    struct CodeState {
        math::RollingMODWT<double> modwt;
        int window_size = 0;
        std::deque<double> log_price_window;

        explicit CodeState(const WaveTrendConfig& cfg);
    };

    WaveTrendConfig _cfg;
    std::vector<int> _window_sizes; ///< 多窗口 MODWT 配置
    std::unordered_map<std::string, CodeState> _states;

    CodeState& ensure_state(const ScopeKey& scope);

    /// 统一价格事件入口（支持 Quote / Tick / Bar）
    void on_price_event(const std::string& code_raw,
                        int64_t ts_ms,
                        double price);

    /// 在窗口 ready() 时计算能量比并发布
    void compute_and_publish(const std::string& scoped_code,
                             CodeState& st,
                             int64_t ts_ms);
};

} // namespace factorlib
