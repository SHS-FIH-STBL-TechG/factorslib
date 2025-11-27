#pragma once
/**
 * @file volume_mgf_factor.h
 * @brief 成交量矩生成函数（MGF）因子（量因子 #68 的工程化实现）。
 *
 * 核心定义：
 *   对窗口内成交量序列 {v_i}，估计矩生成函数
 *       M_X(t) = E[e^{t X}]  ≈ (1/N) Σ exp(t * v_i)
 *
 * 实现说明：
 *   - 使用固定窗口（window_size）保留最近 N 个成交量；
 *   - 同时维护 Σ exp(t * v_i) 的滑动和；
 *   - 每次新成交量到来时 O(1) 更新（减去最老样本的贡献，加上最新样本）。
 *
 * 参数：
 *   - window_size：滑窗长度；
 *   - t          ：MGF 的评估点；
 */

#include <string>
#include <unordered_map>
#include <deque>
#include <vector>

#include "core/ifactor.h"
#include "core/types.h"
#include "core/databus.h"
#include "utils/log.h"

namespace factorlib {

// =====================[ 配置 ]=====================

struct VolumeMGFConfig {
    int   window_size = 256;   ///< 滑窗长度
    double t          = 0.01;  ///< MGF 评估点 t，t>0 越大越关注大体量
};

// =====================[ 主题名 ]=====================

inline constexpr const char* TOP_VOL_MGF = "volume/mgf";

// =====================[ 因子类 ]=====================

class VolumeMGFFactor : public BaseFactor {
public:
    VolumeMGFFactor(const std::vector<std::string>& codes,
                    const VolumeMGFConfig& cfg = {});

    /// 注册 DataBus topic
    static void register_topics(size_t capacity = 1024);

    // IFactor 接口
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
        std::deque<double> window;  ///< 最近 window_size 个成交量
        int   max_size = 0;
        double sum_exp = 0.0;       ///< Σ exp(t * v_i)

        CodeState() = default;
        explicit CodeState(const VolumeMGFConfig& cfg)
            : max_size(cfg.window_size) {}
    };

    VolumeMGFConfig _cfg;
    std::vector<int> _window_sizes; ///< 来自 INI 的多窗口列表
    std::unordered_map<std::string, CodeState> _states;

    CodeState& ensure_state(const ScopeKey& scope);

    /// 统一成交量事件入口
    void on_volume_event(const std::string& code_raw,
                         int64_t ts_ms,
                         double volume);

    /// 计算 MGF 并发布
    void compute_and_publish(const std::string& scoped_code,
                             CodeState& S,
                             int64_t ts_ms);
};

} // namespace factorlib
