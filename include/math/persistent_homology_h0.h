#pragma once
/**
 * @file persistent_homology_h0.h
 * @brief 1D 信号的 0 维持续同调（H0，连通分量）近似实现。
 *
 * 是否增量：❌（当前实现为窗口内一次性重算）
 * 优化策略：
 *  - 使用单链聚类（single-linkage）与并查集（union-find）在半径阈值 ε 下合并相邻点，
 *    对于 1D 信号，先按数值排序后仅检查相邻对，复杂度 O(W log W)；
 *  - 若需要滑窗接近“增量”，建议对窗口做**分层降采样**或**价格网格化**，只对发生变化的桶做局部重算。
 */
#include <vector>
#include <algorithm>
#include <cmath>

namespace factorlib { namespace math {

inline double mean_persistence_h0_1d(const std::vector<double>& x, double eps_max){
    if (x.size()<2) return 0.0;
    std::vector<double> s = x;
    std::sort(s.begin(), s.end());
    std::vector<double> gaps;
    gaps.reserve(s.size()-1);
    for (size_t i=1;i<s.size();++i){
        double g = 0.5*std::fabs(s[i]-s[i-1]);
        if (g<=eps_max) gaps.push_back(g);
    }
    if (gaps.empty()) return 0.0;
    double sum=0.0; for(double g: gaps) sum+=g;
    return sum/static_cast<double>(gaps.size());
}

}} // namespace factorlib::math
