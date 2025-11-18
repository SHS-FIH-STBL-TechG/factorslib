#pragma once
/**
 * @file src/stat_factors/pca_angular_momentum_factor.h
 * @brief 【空因子 #38】多尺度主成分角动量因子（简化实现）。
 *
 * 定义（简化）：
 *   对特征向量 x_t 做在线 PCA，取 PC1 的特征值 λ₁ 作为“质量 m”，
 *   取状态向量 r_t = 均值向量 μ_t，速度 v_t ≈ μ_t - μ_{t-1}，
 *   定义角动量标量 L = m * || r_t × v_t ||（三维向量叉乘模长）。
 *   L 越大表示“旋转运动”越强。
 *
 * 参考：factor_design.md（空因子-多尺度主成分角动量）fileciteturn0file0
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

struct PCAAngMomCfg {
    int    dims = 3;
    int    k    = 1;
    double lr   = 0.05;
    bool   debug_mode=false;
};

inline constexpr const char* TOP_PCA_L = "space/pca_angmom";

class PCAAngularMomentumFactor : public BaseFactor {
public:
    explicit PCAAngularMomentumFactor(const std::vector<std::string>& codes,
                                      const PCAAngMomCfg& cfg = {});
    static void register_topics(size_t capacity=2048) {
        DataBus::instance().register_topic<double>(TOP_PCA_L, capacity);
    }

    void on_tick(const CombinedTick& x) override;
    void on_quote(const QuoteDepth& q) override;
    bool force_flush(const std::string& code) override { (void)code; return false; }

private:
    struct CodeState {
        bool has_last_price=false;
        double last_price=0.0;
        std::vector<double> last_mean;
        math::OnlinePCA pca{3,1,0.05};
    };
    PCAAngMomCfg _cfg;
    std::unordered_map<std::string, CodeState> _states;

    void ensure_state(const std::string& code);
    void feed_vec(const std::string& code, int64_t ts, const std::vector<double>& x);
    void maybe_publish(const std::string& code, int64_t ts);
};

} // namespace factorlib
