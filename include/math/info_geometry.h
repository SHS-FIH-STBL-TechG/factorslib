#pragma once
/**
 * @file info_geometry.h
 * @brief 信息几何：Fisher 度量（Bernoulli / 多项式近似）。
 *
 * 核心概念：
 * - Fisher信息矩阵：描述概率分布参数空间的曲率，在信息几何中作为黎曼度量
 * - Bernoulli分布：单参数θ，Fisher信息为1/[θ(1-θ)]
 * - 多项式分布：多参数情况，使用对角近似简化计算
 *
 * 是否增量：❌（窗口内一次性重算）
 * 优化策略：结合滑窗直方图增量维护概率，再计算 Fisher。
 * 时间复杂度：Bernoulli-O(1), Multinomial-O(n) 已最优
 */

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm> // 添加algorithm头文件用于std::transform
#include <iterator>

namespace factorlib { namespace math {

/**
 * @brief 计算伯努利分布的Fisher信息量
 * @param theta 伯努利分布的成功概率参数，范围(0,1)
 * @return double Fisher信息量 I(θ) = 1/[θ(1-θ)]
 *
 * @note 当θ接近0或1时，Fisher信息量趋于无穷大，表示参数估计的不确定性极小
 * @note 时间复杂度：O(1)，已是最优
 */
inline double fisher_bernoulli(double theta) {
    // 参数边界检查：θ必须在(0,1)开区间内
    if (theta <= 0.0 || theta >= 1.0)
        return std::numeric_limits<double>::infinity();

    // 计算 Fisher 信息：I(θ) = 1/[θ(1-θ)]
    return 1.0 / (theta * (1.0 - theta));
}

/**
 * @brief 计算多项式分布的Fisher信息矩阵对角近似
 * @param p 概率向量，满足∑p_i = 1, p_i ≥ 0
 * @return std::vector<double> Fisher信息矩阵对角元素，g_ii ≈ 1/p_i
 *
 * @note 该方法使用对角近似简化计算，适用于高维情况
 * @note 当p_i=0时返回无穷大，表示该方向参数不可估计
 * @note 时间复杂度：O(n)，已接近最优，使用STL算法微优化
 */
inline std::vector<double> fisher_multinomial_diag(const std::vector<double>& p) {
    std::vector<double> g;
    g.reserve(p.size()); // 预分配空间避免重新分配

    // 使用STL变换算法，可能利用向量化优化
    std::transform(p.begin(), p.end(), std::back_inserter(g),
        [](double prob) -> double {
            return (prob > 0.0) ? (1.0 / prob) : std::numeric_limits<double>::infinity();
        });

    return g;
}

/**
 * @brief 多项式分布Fisher信息的优化版本（避免向量重新分配）
 * @param p 输入概率向量
 * @param[out] g 输出Fisher信息向量（预先分配好空间）
 *
 * @note 性能优化版本，避免内存分配开销
 * @note 调用者需确保g有足够容量：g.resize(p.size())
 */
inline void fisher_multinomial_diag_inplace(const std::vector<double>& p, std::vector<double>& g) {
    // 安全检查：确保输出向量有足够空间
    if (g.size() < p.size()) {
        g.resize(p.size());
    }

    // 原地计算，避免拷贝
    for (size_t i = 0; i < p.size(); ++i) {
        g[i] = (p[i] > 0.0) ? (1.0 / p[i]) : std::numeric_limits<double>::infinity();
    }
}

}} // namespace factorlib::math