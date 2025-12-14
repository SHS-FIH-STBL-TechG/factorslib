#pragma once
/**
 * @file hmm_uptrend_prob_factor.h
 * @brief 两状态高斯 HMM 的上行状态后验概率因子。
 *
 * 理论依据：
 *   - 把收益序列看成由"隐含状态"驱动：s_t ∈ {U, D}（上行/下行制度）
 *   - 条件发射：r_t | s_t ~ N(μ_{s_t}, σ_{s_t}^2)
 *   - 转移：P(s_t = j | s_{t-1} = i) = A_{ij}
 *   - 输出过滤概率 γ_t = P(s_t = U | r_{1:t}) 以及下一期漂移预测
 *
 * 计算流程：
 *   - 使用增量缩放前向算法（来自 math::RollingHMMForward）
 *   - 每步 O(1) 更新滤波状态
 *   - 发射密度：b_j(r_t) = (1 / (√(2π) σ_j)) * exp(-(r_t - μ_j)^2 / (2 σ_j^2))
 */

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/databus.h"
#include "core/ifactor.h"
#include "core/types.h"
#include "math/rolling_hmm_forward.h"

namespace factorlib {

/**
 * @brief HMM 上行状态后验概率因子配置。
 *
 * 主要参数：
 *   - window_size : 滚动窗口长度 N
 *   - mu_up / sigma_up : 上行状态的均值和标准差
 *   - mu_down / sigma_down : 下行状态的均值和标准差
 *   - trans_uu / trans_dd : 状态转移概率 P(U→U) 和 P(D→D)
 */
struct HmmUptrendProbConfig {
    int window_size = 40;           ///< 滚动窗口长度

    // HMM 参数（实盘应使用滚动 EM/Baum-Welch 估计）
    double mu_up     = 0.006;       ///< 上行状态均值
    double sigma_up  = 0.010;       ///< 上行状态标准差
    double mu_down   = -0.006;      ///< 下行状态均值
    double sigma_down = 0.012;      ///< 下行状态标准差

    // 转移矩阵：A = [[trans_uu, 1-trans_uu], [1-trans_dd, trans_dd]]
    double trans_uu = 0.90;         ///< P(U → U)
    double trans_dd = 0.90;         ///< P(D → D)

    // 初始状态概率
    double pi_up = 0.5;             ///< 初始为上行状态的概率

    /**
     * @brief 转换为数学库的 HMM 参数
     */
    math::HMM2GaussianParams to_math_params() const {
        return math::HMM2GaussianParams{
            mu_up, sigma_up,
            mu_down, sigma_down,
            trans_uu, trans_dd,
            pi_up
        };
    }
};

/**
 * @brief 两状态高斯 HMM 的上行状态后验概率因子。
 *
 * 因子含义：
 *   - 使用收盘价构造对数收益率序列 r_t = log(C_t / C_{t-1})
 *   - 使用增量前向算法（O(1) 每步）计算过滤概率
 *   - 输出因子值为当前处于上行制度的后验概率 γ_T ∈ [0, 1]
 *
 * 计算特性：
 *   - 使用 math::RollingHMMForward 进行增量计算
 *   - 每来一根新 K 线，O(1) 更新滤波状态
 */
class HmmUptrendProbFactor : public BaseFactor {
public:
    using Code = std::string;

    /**
     * @brief 构造函数。
     *
     * @param codes 需要计算该因子的证券代码列表
     * @param cfg   因子配置
     */
    HmmUptrendProbFactor(
        const std::vector<Code>& codes,
        const HmmUptrendProbConfig& cfg = HmmUptrendProbConfig{}
    );

    /**
     * @brief 在 DataBus 上注册本因子所需的 topic。
     *
     * topic：
     *   - "kline_new/hmm_uptrend_prob" : 上行状态后验概率
     *   - "kline_new/hmm_next_drift"   : 下一期漂移预测
     */
    static void register_topics(std::size_t capacity);

    /**
     * @brief 使用单根 K 线驱动因子更新。
     */
    void on_bar(const Bar& b) override;

    void on_quote(const QuoteDepth&) override {}
    void on_tick(const CombinedTick&) override {}

    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    /**
     * @brief 单只标的在本因子下的状态。
     *
     * 使用 math::RollingHMMForward 进行增量前向滤波。
     */
    struct CodeState {
        explicit CodeState(const HmmUptrendProbConfig& cfg);

        bool has_last_close;         ///< 是否已记录上一根 K 线收盘价
        double last_close;           ///< 上一根 K 线的收盘价
        math::RollingHMMForward<double> hmm;  ///< 增量 HMM 滤波器

        void push_bar(const Bar& b);
        bool ready() const { return hmm.ready(); }
    };

    HmmUptrendProbConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
