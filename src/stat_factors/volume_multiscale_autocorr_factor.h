#pragma once
/**
 * @file volume_multiscale_autocorr_factor.h
 * @brief 成交量多尺度相关性因子（量因子 #30 的简化实现）。
 *
 * 核心思想：
 *   - 对同一标的的成交量序列 v_t，构造多个固定滞后 k 的 RollingAutoCorr（如 k=1,2,4,8）。
 *   - 在每个时间点，取这些 r_k 的平均值，作为“多尺度相关性一致性”指标：
 *         C_multi = (r_1 + r_2 + r_4 + r_8) / 4
 *
 * 设计目标：
 *   - 高频场景下 O(1) 增量更新；
 *   - 支持窗口大小等参数通过 runtime_config.ini 配置；
 *   - 通过 DataBus 发布单一 double 因值。
 */

#include <string>
#include <unordered_map>
#include <vector>

#include "ifactor.h"
#include "utils/types.h"
#include "utils/databus.h"
#include "utils/log.h"
#include "math/rolling_autocorr.h"

namespace factorlib {

// =====================[ 配置 ]=====================

struct VolumeMultiscaleAutocorrConfig {
    int  window_size = 128;  ///< 滑窗长度（样本数）
    int  lag1        = 1;    ///< 第一层滞后
    int  lag2        = 2;    ///< 第二层滞后
    int  lag3        = 4;    ///< 第三层滞后
    int  lag4        = 8;    ///< 第四层滞后
};

// =====================[ 主题名 ]=====================

inline constexpr const char* TOP_VOL_MULTI_AC = "volume/autocorr_multiscale";

// =====================[ 因子类 ]=====================

class VolumeMultiscaleAutocorrFactor : public BaseFactor {
public:
    VolumeMultiscaleAutocorrFactor(const std::vector<std::string>& codes,
                                   const VolumeMultiscaleAutocorrConfig& cfg = {});

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
        bool ready = false;
        math::RollingAutoCorr<double> ac1;
        math::RollingAutoCorr<double> ac2;
        math::RollingAutoCorr<double> ac3;
        math::RollingAutoCorr<double> ac4;
        int window_size = 0;

        explicit CodeState(const VolumeMultiscaleAutocorrConfig& cfg);
    };

    VolumeMultiscaleAutocorrConfig _cfg;
    std::vector<int> _window_sizes; ///< 允许一次注册多组滑窗
    std::unordered_map<std::string, CodeState> _states;

    CodeState& ensure_state(const ScopeKey& scope);

    /// 统一成交量事件入口
    void on_volume_event(const std::string& code_raw,
                         int64_t ts_ms,
                         double volume);

    /// 计算多尺度相关性并发布
    void compute_and_publish(const std::string& scoped_code,
                             CodeState& S,
                             int64_t ts_ms);
};

} // namespace factorlib
