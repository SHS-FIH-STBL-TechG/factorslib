#pragma once
/**
 * @file memory_kernel.h
 * @brief 记忆核 M(τ) = sum_{k=0}^{L} w_k(α) * ρ(k) 的滑窗估计；ρ(k) 为滞后 k 的自相关。
 *
 * 设计约定：
 *   - ρ(k) 全部来自 RollingAutoCorr 的数值估计；
 *   - 当序列方差为 0 时，RollingAutoCorr::value() 返回 0（自相关“未定义”但数值上视为 0）；
 *   - 因此，对常数序列 {x_t ≡ c}，我们希望所有 ρ(k) ≈ 0，从而 M(τ) ≈ 0。
 *
 * 实现说明：
 *   - 使用 FractionalDifferentiatorGL 只负责生成 Grünwald–Letnikov 权重 w_k(α)；
 *   - 对每个 lag k=1..L 维护一个 RollingAutoCorr 实例；
 *   - 每次 push(x) 时，将原始序列喂给所有自相关估计器；
 *   - value() 时按 M = sum_{k=1}^L w_k * ρ_k 计算记忆核。
 *
 * 复杂度：
 *   - 每次更新：O(L)
 *   - 获取当前 M：O(L)
 */

#include <vector>
#include <cstddef>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "math/rolling_autocorr.h"
#include "math/bad_value_policy.h"
#include "math/fractional_diff.h"

namespace factorlib { namespace math {

template<typename T = double, typename BadPolicy = NoCheckBadValuePolicy>
class MemoryKernelEstimator {
public:
    /**
     * @param W     滑窗长度（参与自相关估计的窗口大小）
     * @param L     记忆阶数（最大的滞后阶）
     * @param alpha 分数阶参数 α（用于构造权重 w_k(α)）
     *
     * 要求：L+1 < W，保证在窗口内能看到足够多的 (x_t, x_{t+k}) 配对。
     */
    MemoryKernelEstimator(std::size_t W, std::size_t L, double alpha)
        : W_(W), L_(L), frac_(alpha, L) {
        if (L_ + 1 >= W_) {
            throw std::invalid_argument("MemoryKernelEstimator: require L+1 < W to have pairs");
        }
        // 预建每个 lag 的自相关估计器：k=1..L
        for (std::size_t k = 1; k <= L_; ++k) {
            ac_.emplace_back(W_, static_cast<int>(k));
        }
    }

    /**
     * @brief 推入一个新样本。
     *
     * 坏值处理：
     *   - 若 T 为浮点类型，则内部 RollingAutoCorr 会使用 BadPolicy 进行数值过滤；
     *   - 这里直接把 x 原样传下去，由 BadPolicy 决定是否接收。
     *
     * @return true 表示所有自相关估计器都成功更新；若有 BadPolicy 拒绝则返回 false。
     */
    bool push(T x) {
        bool ok = true;
        for (auto& ac : ac_) {
            ok = ac.push(x) && ok;
        }
        return ok;
    }

    /// 是否已经有足够数据让所有自相关估计器都 ready()
    bool ready() const {
        if (ac_.empty()) return false;
        for (const auto& ac : ac_) {
            if (!ac.ready()) return false;
        }
        return true;
    }

    /**
     * @brief 计算当前记忆核值 M(τ)。
     *
     * 约定：
     *   - 若任一自相关估计器未 ready()，返回 NaN；
     *   - 否则按 M = Σ_{k=1}^L w_k(α) · ρ(k) 计算；
     *   - 不再显式加入 ρ(0)=1 项，常数序列时 ρ(k>0)≈0 ⇒ M≈0。
     */
    double value() const {
        if (!ready()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const auto& w = frac_.weights();
        long double M = 0.0L;
        for (std::size_t k = 1; k <= L_; ++k) {
            double rho = ac_[k - 1].value();
            M += static_cast<long double>(w[k]) * static_cast<long double>(rho);
        }
        return static_cast<double>(M);
    }

private:
    std::size_t W_;
    std::size_t L_;
    FractionalDifferentiatorGL<T, BadPolicy> frac_;
    std::vector< RollingAutoCorr<T, BadPolicy> > ac_;
};

}} // namespace factorlib::math
