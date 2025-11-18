#pragma once
/**
 * @file pca_structure_stability_factor.h
 * @brief 【空因子 #17】主成分结构稳定性因子（PC1 方向变化角）。
 *
 * 定义：
 *   利用增量 PCA（Oja）对 d 维向量 x_t 做在线主成分提取；
 *   记当前与上一步的一主成分向量 v_t, v_{t-1}，定义稳定性：
 *       S = cos(∠(v_t, v_{t-1})) = <v_t, v_{t-1}>
 *   取值 ∈ [-1,1]，越接近 1 说明结构越稳定。
 *
 * 特征向量构造（默认三维）：
 *   x_t = [对数收益, 成交量, OFI]ᵀ，其中 OFI≈买量-卖量（逐笔成交方向近似）。
 *
 * 参考：factor_design.md（空因子-主成分结构稳定性）fileciteturn0file0
 */

#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>

#include "ifactor.h"
#include "utils/types.h"
#include "utils/databus.h"
#include "utils/log.h"
#include "math/online_pca.h"
#include "../config/runtime_config.h"

namespace factorlib {

struct PCAStabilityCfg {
    int    dims = 3;     ///< 特征维度，默认 3（return, volume, ofi）
    int    k    = 1;     ///< 主成分数，只需 1
    double lr   = 0.05;  ///< Oja 学习率
    bool   debug_mode=false;
};

inline constexpr const char* TOP_PCA_STAB = "space/pca_stability";

class PCAStructureStabilityFactor : public BaseFactor {
public:
    explicit PCAStructureStabilityFactor(const std::vector<std::string>& codes,
                                         const PCAStabilityCfg& cfg = {});
    static void register_topics(size_t capacity=2048) {
        DataBus::instance().register_topic<double>(TOP_PCA_STAB, capacity);
    }

    void on_tick(const CombinedTick& x) override;
    void on_quote(const QuoteDepth& q) override;
    bool force_flush(const std::string& code) override { (void)code; return false; }

private:
    struct CodeState {
        bool has_last_price=false;
        double last_price=0.0;
        double last_v1_angle=std::numeric_limits<double>::quiet_NaN();
        double ofi_acc=0.0;   ///< 本 tick 累计 OFI（逐笔）
        math::OnlinePCA pca{3,1,0.05};
    };
    PCAStabilityCfg _cfg;
    std::unordered_map<std::string, CodeState> _states;

    void ensure_state(const std::string& code);
    void feed_vec(const std::string& code, int64_t ts, const std::vector<double>& x);
    void maybe_publish(const std::string& code, int64_t ts);
};

} // namespace factorlib
