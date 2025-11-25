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
 *
 * 分数阶差分是整数阶差分的推广，能够描述具有记忆效应的系统。
 * 在金融时间序列分析中，分数阶差分可以更好地保留长期记忆特性。
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

/**
 * @class FractionalDifferentiatorGL
 * @brief Grünwald-Letnikov 分数阶差分器
 *
 * @tparam T 数据类型，通常为 float 或 double
 * @tparam BadPolicy 坏值处理策略，默认不检查
 *
 * 该类实现了基于 Grünwald-Letnikov 定义的分数阶差分计算，
 * 采用滑动窗口机制进行增量计算，适合实时数据处理。
 */
template<typename T=float, typename BadPolicy=NoCheckBadValuePolicy>
class FractionalDifferentiatorGL {
public:
    using value_type = T;

    /**
     * @brief 构造函数
     * @param alpha 分数阶阶数，通常在 (0,1) 范围内
     * @param L 记忆长度（最大滞后阶数），决定了计算精度
     *
     * 记忆长度 L 越大，计算越精确但计算量也越大。
     * 实际应用中需要权衡精度和计算效率。
     */
    FractionalDifferentiatorGL(double alpha, std::size_t L)
        : alpha_(alpha), L_(L) {
        if (L_ == 0) L_ = 1;  // 记忆长度至少为1
        build_weights();      // 预计算权重系数
        buf_.assign(L_+1, T{});  // 初始化缓冲区
        filled_ = 0;          // 标记缓冲区尚未填满
    }

    /**
     * @brief 推入新样本数据
     * @param x 新输入的数据点
     * @return 是否成功推入（如果策略过滤了坏值则返回false）
     *
     * 该方法采用环形缓冲区管理数据，每次推入新数据时会覆盖最旧的数据。
     * 时间复杂度：O(1)
     */
    bool push(T x) {
        // 检查输入数据的有效性（仅对浮点类型）
        if constexpr (std::is_floating_point_v<T>) {
            if (!is_finite_numeric(x)) {
                // 使用坏值处理策略处理异常值
                if (!BadPolicy::handle(x, "FractionalDifferentiatorGL::push")) {
                    // 策略拒绝该样本（如选择丢弃）
                    return false;
                }
            }
        }

        // 确保缓冲区大小正确
        if (buf_.size() < L_+1) buf_.resize(L_+1, T{});

        // 更新环形缓冲区头部指针
        head_ = (head_ + 1) % (L_+1);
        buf_[head_] = x;  // 存储新数据

        // 更新填充状态
        if (filled_ < L_+1) ++filled_;
        return true;
    }

    /**
     * @brief 检查是否已准备好计算分数阶差分
     * @return 如果缓冲区已填满（拥有 L+1 个样本）则返回 true
     *
     * 只有缓冲区填满时，计算的结果才是基于完整记忆长度的准确值。
     */
    bool ready() const { return filled_ >= (L_+1); }

    /**
     * @brief 计算当前的分数阶差分值
     * @return 分数阶差分结果 D^α x_t
     *
     * 计算复杂度：O(L)，其中 L 为记忆长度。
     * 如果缓冲区未填满，返回 NaN。
     *
     * 计算公式：D^α x_t = Σ_{k=0}^L w_k(α) * x_{t-k}
     * 其中 w_k 为预先计算的 Grünwald-Letnikov 权重系数。
     */
    T value() const {
        if (!ready()) return std::numeric_limits<T>::quiet_NaN();

        long double acc = 0.0L;  // 使用 long double 提高计算精度
        // 遍历所有滞后项，计算加权和
        for (std::size_t k = 0; k <= L_; ++k) {
            // 从环形缓冲区获取滞后 k 步的数据
            // 注意：head_ 指向最新数据，所以需要反向索引
            const T& x = buf_[(head_ + (L_+1) - k) % (L_+1)];
            acc += static_cast<long double>(weights_[k]) * static_cast<long double>(x);
        }
        return static_cast<T>(acc);
    }

    /**
     * @brief 获取权重系数向量
     * @return 权重系数向量的常量引用
     *
     * 权重系数可用于外部分析、能量计算或其他数学操作。
     */
    const std::vector<long double>& weights() const { return weights_; }

    /// @brief 获取记忆长度
    std::size_t memory() const { return L_; }

    /// @brief 获取分数阶阶数
    double alpha() const { return alpha_; }

private:
    /**
     * @brief 构建 Grünwald-Letnikov 权重系数
     *
     * 权重计算公式：
     *   w_0 = 1
     *   w_k = w_{k-1} * ((α - (k-1)) / k) * (-1), k = 1,2,...,L
     *
     * 这些权重系数体现了分数阶微积分的记忆特性：
     * - 权重衰减速度比整数阶差分慢
     * - 所有历史数据都对当前差分有贡献，贡献程度随时间衰减
     */
    void build_weights() {
        weights_.assign(L_+1, 0.0L);
        weights_[0] = 1.0L;  // 当前时刻的权重为1

        // 递归计算各阶权重
        for (std::size_t k = 1; k <= L_; ++k) {
            long double prev = weights_[k-1];  // 前一阶权重
            // 计算权重更新因子
            long double term = ( (alpha_ - static_cast<long double>(k-1)) / static_cast<long double>(k) );
            weights_[k] = - prev * term; // 乘以 (-1) 得到交替符号
        }
    }

private:
    double alpha_;                         ///< 分数阶阶数
    std::size_t L_;                        ///< 记忆长度
    std::vector<long double> weights_;     ///< Grünwald-Letnikov 权重系数
    std::vector<T> buf_;                   ///< 环形缓冲区，存储最近 L+1 个数据点
    std::size_t head_{static_cast<std::size_t>(-1)};  ///< 缓冲区头部指针（指向最新数据）
    std::size_t filled_{0};                ///< 缓冲区中已填充的数据数量
};

}} // namespace factorlib::math