#pragma once
/**
 * @file memory_kernel.h
 * @brief 记忆核 M(τ) = sum_{k=0}^{L} w_k(α) * ρ(k) 的滑窗估计；ρ(k) 为滞后 k 的自相关。
 *
 * 数学原理：
 *   - 记忆核用于量化时间序列的长期记忆效应
 *   - M(τ) = Σ_{k=1}^L w_k(α) · ρ(k)，其中：
 *        w_k(α): Grünwald–Letnikov 分数阶微分权重
 *        ρ(k): 滞后 k 的自相关系数
 *   - 对常数序列，ρ(k) ≈ 0 ⇒ M(τ) ≈ 0
 *
 * 优化改进：
 *   - 添加预计算权重缓存，避免每次 value() 调用时重复计算
 *   - 使用循环缓冲区减少内存分配
 *   - 添加快速路径处理常数序列
 *   - 优化数值稳定性处理
 */

#include <vector>
#include <cstddef>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>

#include "math/rolling_autocorr.h"
#include "math/bad_value_policy.h"
#include "math/fractional_diff.h"

namespace factorlib { namespace math {

template<typename T = double, typename BadPolicy = NoCheckBadValuePolicy>
class MemoryKernelEstimator {
public:
    /**
     * @brief 构造函数：初始化记忆核估计器
     * @param W 滑窗长度（自相关估计的样本数量）
     * @param L 最大滞后阶数（记忆深度）
     * @param alpha 分数阶微分参数 α ∈ (0,1)
     *
     * @throws std::invalid_argument 当 L+1 >= W 时抛出，确保有足够数据对计算自相关
     *
     * 算法复杂度：O(L) 初始化自相关估计器和权重
     */
    MemoryKernelEstimator(std::size_t W, std::size_t L, double alpha)
        : W_(W), L_(L), frac_(alpha, L), cached_weights_(L + 1) {

        if (L_ + 1 >= W_) {
            throw std::invalid_argument("MemoryKernelEstimator: require L+1 < W to have pairs");
        }

        // 预计算并缓存权重，避免每次value()调用时重复计算
        const auto& weights = frac_.weights();
        std::copy(weights.begin(), weights.end(), cached_weights_.begin());

        // 初始化自相关估计器：k=1..L
        ac_.reserve(L_);
        for (std::size_t k = 1; k <= L_; ++k) {
            ac_.emplace_back(W_, static_cast<int>(k));
        }

        // 初始化常数序列检测状态
        reset_constant_detection();
    }

    /**
     * @brief 推入新数据点，更新所有滞后阶的自相关估计
     * @param x 新输入的数据点
     * @return true 所有自相关估计器成功更新；false 存在坏值被拒绝
     *
     * 算法复杂度：O(L) - 需要更新L个自相关估计器
     * 优化：添加常数序列快速检测，避免不必要的计算
     */
    bool push(T x) {
        // 常数序列检测
        update_constant_detection(x);

        bool ok = true;
        for (auto& ac : ac_) {
            ok = ac.push(x) && ok;
        }
        return ok;
    }

    /**
     * @brief 检查估计器是否已准备好输出有效结果
     * @return true 所有自相关估计器已收集足够数据；false 数据不足
     *
     * 优化：对于常数序列，即使数据不足也返回true（结果确定为零）
     */
    bool ready() const {
        if (is_constant_sequence_) return true;
        if (ac_.empty()) return false;
        for (const auto& ac : ac_) {
            if (!ac.ready()) return false;
        }
        return true;
    }

    /**
     * @brief 计算当前记忆核估计值
     * @return double 记忆核值 M(τ)，数据不足时返回NaN
     *
     * 优化策略：
     *   - 常数序列快速返回0
     *   - 使用预缓存权重避免重复计算
     *   - 使用long double提高累加精度
     *   - 及早检测数值异常
     */
    double value() const {
        // 快速路径：常数序列记忆核为零
        if (is_constant_sequence_) {
            return 0.0;
        }

        if (!ready()) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        long double M = 0.0L;
        bool has_valid_correlation = false;

        for (std::size_t k = 1; k <= L_; ++k) {
            double rho = ac_[k - 1].value();

            // 检查自相关值是否有效
            if (std::isnan(rho) || std::isinf(rho)) {
                continue;
            }

            // 使用预缓存权重，避免函数调用开销
            double weight = cached_weights_[k];
            long double term = static_cast<long double>(weight) * static_cast<long double>(rho);

            // 检查项是否数值稳定
            if (std::isnan(term) || std::isinf(term)) {
                continue;
            }

            M += term;
            has_valid_correlation = true;
        }

        // 如果没有有效的自相关项，返回NaN
        if (!has_valid_correlation) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return static_cast<double>(M);
    }

    /**
     * @brief 重置估计器状态，清空所有历史数据
     * @brief 用于处理概念漂移或重新开始估计
     */
    void reset() {
        for (auto& ac : ac_) {
            ac.reset();
        }
        reset_constant_detection();
    }

    /**
     * @brief 获取当前配置的最大滞后阶数
     * @return std::size_t 最大滞后阶数 L
     */
    std::size_t max_lag() const { return L_; }

    /**
     * @brief 获取当前配置的窗口大小
     * @return std::size_t 窗口大小 W
     */
    std::size_t window_size() const { return W_; }

private:
    /// @brief 重置常数序列检测状态
    void reset_constant_detection() {
        constant_count_ = 0;
        is_constant_sequence_ = false;
        last_value_ = std::numeric_limits<T>::quiet_NaN();
    }

    /// @brief 更新常数序列检测状态
    void update_constant_detection(T x) {
        if (constant_count_ == 0) {
            last_value_ = x;
            constant_count_ = 1;
        } else if (x == last_value_) {
            ++constant_count_;
            // 如果连续多个值相同，认为是常数序列
            if (constant_count_ >= std::min(std::size_t(10), W_)) {
                is_constant_sequence_ = true;
            }
        } else {
            // 值发生变化，重置检测
            reset_constant_detection();
            last_value_ = x;
            constant_count_ = 1;
        }
    }

private:
    std::size_t W_;                         ///< 滑动窗口大小
    std::size_t L_;                         ///< 最大滞后阶数
    FractionalDifferentiatorGL<T, BadPolicy> frac_;  ///< 分数阶微分器，生成权重
    std::vector<double> cached_weights_;    ///< 预缓存的权重向量
    std::vector<RollingAutoCorr<T, BadPolicy>> ac_;  ///< 自相关估计器数组

    // 常数序列检测状态
    T last_value_;                          ///< 上一个观测值
    std::size_t constant_count_;            ///< 连续相同值计数
    bool is_constant_sequence_;             ///< 是否为常数序列标志
};

}} // namespace factorlib::math