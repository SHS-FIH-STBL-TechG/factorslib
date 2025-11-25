#pragma once
/**
 * @file hawkes_intensity.h
 * @brief 离散时间下的 Hawkes 过程强度在线估计：
 *        λ_{t+1} = μ + e^{-βΔt} (λ_t - μ) + α * n_t
 * 其中 n_t 为当前步事件数（或 0/1）。用于因子 #11, #31, #71。
 *
 * Hawkes 过程是一种自激励点过程，广泛应用于金融高频数据分析：
 * - 在订单流、交易爆发、市场微观结构分析中特别有用
 * - 能够捕捉事件的聚集效应（一个事件的发生会增加后续事件发生的概率）
 * - 适用于波动率聚集、交易传染等市场现象建模
 */
#include <cmath>
#include <limits>

namespace factorlib {
namespace math {

/**
 * @class HawkesIntensity
 * @brief Hawkes 过程强度在线估计器
 *
 * 实现离散时间版本的 Hawkes 过程强度更新：
 *   λ_{t+1} = μ + e^{-βΔt} (λ_t - μ) + α * n_t
 *
 * 数学模型解释：
 * - μ (mu): 基础强度，表示事件的背景发生率
 * - α (alpha): 自激励系数，表示单个事件对强度的瞬时提升
 * - β (beta): 衰减速率，控制历史影响的衰减速度
 * - λ_t: 当前时刻的强度值
 * - n_t: 当前时刻发生的事件数量（高频场景下通常为0或1）
 *
 * 应用场景：
 * - 因子 #11: 订单流自激励效应
 * - 因子 #31: 交易爆发的持续时间预测
 * - 因子 #71: 市场微观结构的聚集效应
 */
class HawkesIntensity {
public:
    /**
     * @brief 构造函数
     * @param mu 基础强度 μ > 0，代表事件的背景发生率
     * @param alpha 自激励系数 α ≥ 0，代表单个事件对强度的提升
     * @param beta 衰减速率 β > 0，控制历史影响的指数衰减
     * @param dt 时间步长，默认1.0（单位时间）
     *
     * 参数约束说明：
     * - 为保证过程平稳，通常要求 α < β
     * - 初始强度 λ_0 设为 μ（过程从稳态开始）
     * - 如果 α ≥ β，过程可能发散（强度无限增长）
     */
    HawkesIntensity(double mu, double alpha, double beta, double dt=1.0)
        : mu_(mu), alpha_(alpha), beta_(beta), dt_(dt), lambda_(mu) {
        // 注意：这里没有进行参数有效性检查，使用者需确保参数合理
    }

    /**
     * @brief 更新 Hawkes 过程强度
     * @param n_t 当前时间步发生的事件数量
     * @return 更新后的强度值 λ_{t+1}
     *
     * 更新公式分解：
     * 1. 均值回归项: μ + e^{-βΔt} (λ_t - μ)
     *    - 强度 λ_t 以速率 β 向基础强度 μ 衰减
     *    - e^{-βΔt} 是衰减因子，Δt 越大衰减越快
     * 2. 自激励项: α * n_t
     *    - 每个发生的事件立即增加强度 α
     *    - 在金融高频数据中，n_t 通常为 0（无交易）或 1（有交易）
     *
     * 计算复杂度: O(1)
     */
    double update(double n_t) {
        double decay = std::exp(-beta_ * dt_);  // 计算指数衰减因子
        lambda_ = mu_ + decay * (lambda_ - mu_) + alpha_ * n_t;
        return lambda_;
    }

    /// @brief 获取当前强度值
    double value() const { return lambda_; }

    /// @brief 获取基础强度参数
    double mu() const { return mu_; }

    /// @brief 获取自激励系数
    double alpha() const { return alpha_; }

    /// @brief 获取衰减速率
    double beta() const { return beta_; }

    /// @brief 获取时间步长
    double dt() const { return dt_; }

private:
    double mu_;       ///< 基础强度，代表事件的背景发生率
    double alpha_;    ///< 自激励系数，单个事件对强度的瞬时提升
    double beta_;     ///< 衰减速率，控制历史影响的衰减速度
    double dt_;       ///< 时间步长，离散化参数
    double lambda_;   ///< 当前强度值，反映事件的瞬时发生率
};

}} // namespace factorlib::math