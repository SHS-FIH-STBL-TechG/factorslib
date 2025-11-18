#pragma once
/**
 * @file info_geometry.h
 * @brief 信息几何：Fisher 度量（Bernoulli / 多项式近似）。
 *
 * 是否增量：❌（窗口内一次性重算）
 * 优化策略：结合滑窗直方图增量维护概率，再计算 Fisher。
 */
#include <vector>
#include <cmath>
#include <limits>

namespace factorlib { namespace math {

/// Bernoulli(θ) 的 Fisher 信息：I(θ)=1/[θ(1-θ)]
inline double fisher_bernoulli(double theta){
    if (theta<=0.0 || theta>=1.0) return std::numeric_limits<double>::infinity();
    return 1.0 / (theta*(1.0-theta));
}

/// 多项式分布 Fisher 度量的对角近似：I_ii ≈ 1/p_i（在概率单纯形约束下的近似）
inline std::vector<double> fisher_multinomial_diag(const std::vector<double>& p){
    std::vector<double> g(p.size(), 0.0);
    for (size_t i=0;i<p.size();++i){
        g[i] = (p[i]>0.0) ? (1.0/p[i]) : std::numeric_limits<double>::infinity();
    }
    return g;
}

}} // namespace factorlib::math
