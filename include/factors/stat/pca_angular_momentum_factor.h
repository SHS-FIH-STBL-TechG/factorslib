#pragma once
/**
 * @file pca_angular_momentum_factor.h
 * @brief 多尺度主成分“角动量”因子：在前两主成分子空间中的轨迹弯曲强度。
 *
 * 直观解释：
 *   - 使用 OnlinePCA<double> 在线估计前 k(>=2) 个主成分；
 *   - 将当前特征向量 x_t 投影到前两主成分 (PC1, PC2) 得到坐标 (u_t, v_t)；
 *   - 与上一时刻 (u_{t-1}, v_{t-1}) 形成的二维向量做叉积模长：
 *         L_t = |u_{t-1} * v_t - v_{t-1} * u_t|
 *     近似刻画“运动轨迹”的转弯/旋转强度，越大说明结构变化越剧烈。
 */

#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

#include "core/ifactor.h"
#include "core/types.h"
#include "core/databus.h"
#include "utils/log.h"
#include "math/online_pca.h"

namespace factorlib {

struct PcaAngularMomentumConfig {
    int   dims       = 3;      ///< 特征维度
    int   k          = 2;      ///< 至少 2 个主成分
    double lr        = 0.05;   ///< Oja 学习率
    int   warmup     = 64;     ///< 预热样本数
};

inline constexpr const char* TOP_PCA_ANG = "space/pca_angmom";

class PcaAngularMomentumFactor : public BaseFactor {
public:
    PcaAngularMomentumFactor(const std::vector<std::string>& codes,
                             const PcaAngularMomentumConfig& cfg = {});

    static void register_topics(std::size_t capacity = 1024);

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

        bool   last_proj_valid  = false;
        double last_u           = 0.0;  ///< 上一次在 PC1 上的投影
        double last_v           = 0.0;  ///< 上一次在 PC2 上的投影

        long long n_samples     = 0;

        explicit CodeState(const PcaAngularMomentumConfig& cfg);
    };

    PcaAngularMomentumConfig _cfg;
    std::vector<int> _window_sizes; ///< 桥接 ProcessingAxes + 多窗口配置
    std::unordered_map<std::string, CodeState> _states;

    CodeState& ensure_state(const ScopeKey& scope);

    std::vector<double> make_features(const Bar& b,
                                      double ret) const;

    void on_price_event(const std::string& code_raw,
                        int64_t ts_ms,
                        const Bar& b);

    void maybe_publish(const std::string& scoped_code,
                       CodeState& st,
                       int64_t ts_ms,
                       const std::vector<double>& x);
};

} // namespace factorlib
