#pragma once
/**
 * @file fractional_diff.h
 * @brief Grünwald–Letnikov 分数阶差分（支持滑窗、增量计算）。
 *
 * 设计目标：
 *  - 作为分数阶微积分模块的基础组件，支撑因子 #4, #21, #24 等。
 *  - 每次 push() 只做 O(L) 的权重加权，避免全量重算；L 为记忆长度。
 *  - 支持 BadValuePolicy：默认不处理；可选择丢弃或替换 NaN/Inf。
 *
 * 数学背景：D^α x_t ≈ sum_{k=0}^L w_k(α) * x_{t-k}，其中
 *   w_0 = 1,
 *   w_k = w_{k-1} * ( (α - (k-1)) / k ) * (-1)
 */
#include <vector>
#include <deque>
#include <cmath>
#include <cstddef>
#include <type_traits>

#include "math/bad_value_policy.h"
#include "utils/log.h"

namespace factorlib {
namespace math {

template<typename T=float, typename BadPolicy=NoCheckBadValuePolicy>
class FractionalDifferentiatorGL {
public:
    using value_type = T;

    /**
     * @param alpha 分数阶 α，典型在 (0,1)
     * @param L     记忆长度（最大的滞后阶）
     */
    FractionalDifferentiatorGL(double alpha, std::size_t L)
        : alpha_(alpha), L_(L) {
        if (L_ == 0) L_ = 1;
        build_weights();
        buf_.assign(L_+1, T{});
        filled_ = 0;
    }

    /// 推入新样本；返回是否成功（若策略过滤了坏值返回 false）
    bool push(T x) {
        if constexpr (std::is_floating_point_v<T>) {
            if (!is_finite_numeric(x)) {
                if (!BadPolicy::handle(x, "FractionalDifferentiatorGL::push")) {
                    // 被策略拒绝（如：丢弃该样本）
                    return false;
                }
            }
        }
        // 环形缓冲：把最旧样本弹出（由 deque 自动管理，我们只覆盖写入）
        if (buf_.size() < L_+1) buf_.resize(L_+1, T{});
        // head 指向最新位置
        head_ = (head_ + 1) % (L_+1);
        buf_[head_] = x;
        if (filled_ < L_+1) ++filled_;
        return true;
    }

    /// 当前是否已经拥有 L+1 个样本（即能计算严格的 L 记忆差分）
    bool ready() const { return filled_ >= (L_+1); }

    /// 在当前缓冲下计算 D^α x_t；复杂度 O(L)
    T value() const {
        if (!ready()) return std::numeric_limits<T>::quiet_NaN();
        long double acc = 0.0L;
        // 按权重与滞后做卷积：x_{t-k} = buf_[(head_-k) mod (L+1)]
        for (std::size_t k = 0; k <= L_; ++k) {
            const T& x = buf_[(head_ + (L_+1) - k) % (L_+1)];
            acc += static_cast<long double>(weights_[k]) * static_cast<long double>(x);
        }
        return static_cast<T>(acc);
    }

    /// 访问权重（便于外部做内积、能量等）
    const std::vector<long double>& weights() const { return weights_; }

    std::size_t memory() const { return L_; }
    double alpha() const { return alpha_; }

private:
    void build_weights() {
        weights_.assign(L_+1, 0.0L);
        weights_[0] = 1.0L;
        for (std::size_t k = 1; k <= L_; ++k) {
            long double prev = weights_[k-1];
            long double term = ( (alpha_ - static_cast<long double>(k-1)) / static_cast<long double>(k) );
            weights_[k] = - prev * term; // 乘以 (-1)
        }
    }

private:
    double alpha_;
    std::size_t L_;
    std::vector<long double> weights_;   // w_k
    std::vector<T> buf_;                 // 长度 L+1 的环形缓冲
    std::size_t head_{static_cast<std::size_t>(-1)};
    std::size_t filled_{0};
};

}} // namespace factorlib::math
