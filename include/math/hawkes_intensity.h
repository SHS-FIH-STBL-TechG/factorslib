#pragma once
/**
 * @file hawkes_intensity.h
 * @brief 离散时间下的 Hawkes 过程强度在线估计：
 *        λ_{t+1} = μ + e^{-βΔt} (λ_t - μ) + α * n_t
 * 其中 n_t 为当前步事件数（或 0/1）。用于因子 #11, #31, #71。
 */
#include <cmath>
#include <limits>

namespace factorlib {
namespace math {

class HawkesIntensity {
public:
    HawkesIntensity(double mu, double alpha, double beta, double dt=1.0)
        : mu_(mu), alpha_(alpha), beta_(beta), dt_(dt), lambda_(mu) {}

    /// 传入当前步事件计数 n_t（高频下可理解为 tick 事件是否发生）
    double update(double n_t) {
        double decay = std::exp(-beta_ * dt_);
        lambda_ = mu_ + decay * (lambda_ - mu_) + alpha_ * n_t;
        return lambda_;
    }

    double value() const { return lambda_; }

private:
    double mu_, alpha_, beta_, dt_;
    double lambda_;
};

}} // namespace factorlib::math
