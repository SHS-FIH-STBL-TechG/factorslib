#pragma once
/**
 * @file pca_structure_stability_factor.h
 * @brief 多维状态结构稳定性因子：在线 PCA 的主方向相邻余弦相似度。
 *
 * 思路：
 *   - 对每个标的构造特征向量 x_t（如 [log-return, log(volume+1), log(vwap+1)]）；
 *   - 使用 include/math/online_pca.h 中的 OnlinePCA<double> 做增量 PCA；
 *   - 每步取第一主成分向量 u_t（components()[0]），与上一时刻 u_{t-1} 做余弦相似度：
 *        S_t = | <u_t, u_{t-1}> |，范围 [0,1]，越接近 1 表示结构越稳定；
 *   - 在 DataBus 上发布 S_t。
 *
 * 只使用 OnlinePCA 提供的接口：
 *   - 构造：OnlinePCA<double>(dims, k, lr)
 *   - push(const std::vector<double>&)
 *   - components() const
 */

#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

#include "ifactor.h"
#include "utils/types.h"
#include "utils/databus.h"
#include "utils/log.h"
#include "math/online_pca.h"

namespace factorlib {

struct PcaStructureStabilityConfig {
    int   dims       = 3;      ///< 特征维度（默认 3）
    int   k          = 1;      ///< 主成分个数（当前只用第 1 主成分）
    double lr        = 0.05;   ///< Oja 更新学习率
    int   warmup     = 64;     ///< 最少有效样本数（预热期）
};

inline constexpr const char* TOP_PCA_STAB = "space/pca_stability";

class PcaStructureStabilityFactor : public BaseFactor {
public:
    PcaStructureStabilityFactor(const std::vector<std::string>& codes,
                                const PcaStructureStabilityConfig& cfg = {});

    /// 注册 DataBus topic
    static void register_topics(std::size_t capacity = 1024);

    // IFactor 接口
    void on_quote(const QuoteDepth& /*q*/) override {}
    void on_tick (const CombinedTick& /*x*/) override {}
    void on_bar  (const Bar& b) override;

    bool force_flush(const std::string& /*code*/) override { return false; }

    void on_code_added(const std::string& code) override;

private:
    struct CodeState {
        math::OnlinePCA<double> pca;
        bool   has_last_close   = false;
        double last_close       = 0.0;

        bool   last_pc1_valid   = false;
        std::vector<double> last_pc1;
        long long n_samples     = 0;

        explicit CodeState(const PcaStructureStabilityConfig& cfg);
    };

    PcaStructureStabilityConfig _cfg;
    std::vector<int> _window_sizes; ///< 允许不同窗口长度的结构稳定性并行计算
    std::unordered_map<std::string, CodeState> _states;

    CodeState& ensure_state(const ScopeKey& scope);

    /// 从 Bar 构造特征向量
    std::vector<double> make_features(const Bar& b,
                                      double ret) const;

    /// 统一价格事件入口
    void on_price_event(const std::string& code_raw,
                        int64_t ts_ms,
                        const Bar& b);

    /// 计算结构稳定性并发布
    void maybe_publish(const std::string& scoped_code,
                       CodeState& st,
                       int64_t ts_ms);
};

} // namespace factorlib
