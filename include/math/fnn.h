// math/fnn.h
#pragma once
/**
 * @file fnn.h
 * @brief 虚假最近邻（FNN, False Nearest Neighbors）比例估计（基于 KD-Tree ANN）。
 *
 * 核心思路（离线批量计算）：
 *   - 输入为标量时间序列 x(t)，先做相空间重构：
 *       Y_m(i)     = [x_i, x_{i+tau}, ..., x_{i+(m-1)tau}]        (m 维)
 *       Y_{m+1}(i) = [x_i, x_{i+tau}, ..., x_{i+m\tau}]          (m+1 维)
 *   - 在 m 维空间中，用封装好的 KD-Tree（utils/nn/ann_index.h 内的 KDTreeANN）为
 *     每个点 Y_m(i) 找到最近邻 j(i)（排除自身），然后在原始时间序列上检查
 *     升维到 m+1 维时“新维度”是否导致距离爆炸：
 *
 *       R(i) = |x_{i+m\tau} - x_{j(i)+m\tau}| / || Y_m(i) - Y_m(j(i)) ||
 *
 *     若 R(i) > Rtol，则认为该点在 m 维空间是“虚假最近邻”（False NN）。
 *
 * 是否增量（在线）：❌
 *   - 本实现针对“一段固定长度的时间序列 x[0..N-1]”一次性重构并计算 FNN 比例，
 *     不在 tick 级别维护滑动状态；
 *   - 但在这段固定窗口内部，最近邻搜索已经通过 KD-Tree（基于 nanoflann）
 *     做了复杂度优化，避免了朴素 O(N^2) 的全量暴力搜索。
 *
 * 复杂度（相对于朴素 O(N^2) 的优化）：
 *   - 重构嵌入：O(N · m)
 *       使用 rosenstein_lyapunov.h 中的 embed(x, m, tau) / embed(x, m+1, tau)。
 *   - KD-Tree 构建：O(N log N)
 *       使用 nn::KDTreeANN.build(Y_m)，一次性建立索引。
 *   - 每个点最近邻查询：~O(log N)，总体 ~O(N log N)
 *       对 Y_m(i) 调用 knn(..., k=2) 并排除自身索引。
 *   - 总体复杂度：O(N · m + N log N)，比朴素的 O(N^2 · m) 显著降低，
 *       适合中等长度窗口的离线 FNN 估计。
 */

#include <vector>
#include <cmath>
#include <limits>

#include "utils/nn/ann_index.h"
#include "math/rosenstein_lyapunov.h"  // 复用 embed()

namespace factorlib { namespace math {

/**
 * @brief 计算 FNN 比例：从 m 维提升到 m+1 维时的“虚假最近邻”比例。
 *
 * @param x     标量时间序列（长度为 N）
 * @param m     当前嵌入维度（>=1）
 * @param tau   时间延迟（>0）
 * @param Rtol  虚假最近邻判定阈值，典型取值范围约 10~15
 * @return      虚假最近邻比例；若有效样本为 0 或参数非法则返回 NaN
 */
inline double fnn_ratio(const std::vector<double>& x,
                        int m, int tau,
                        double Rtol)
{
    if (m <= 0 || tau <= 0 ||
        x.size() < static_cast<std::size_t>((m + 1) * tau + 1)) {
        // 序列太短或参数非法，直接返回 NaN
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 利用 rosenstein_lyapunov.h 中的嵌入工具，只做一次实现：
    auto Y_m   = embed(x, m,     tau);
    auto Y_mp1 = embed(x, m + 1, tau);

    const int N_mp1 = static_cast<int>(Y_mp1.size());
    if (N_mp1 <= 1) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 为了对齐时间索引，只使用 Y_m 的前 N_mp1 个点
    std::vector<std::vector<double>> Ym_trunc(N_mp1);
    for (int i = 0; i < N_mp1; ++i) {
        Ym_trunc[i] = std::move(Y_m[i]);
    }

    // 1) 在 m 维嵌入上构建 KD-Tree 索引（基于 nanoflann 的封装）
    factorlib::nn::KDTreeANN index;
    index.build(Ym_trunc);

    // 2) 遍历每个点 i，找到 m 维最近邻 j(i)，再在 m+1 维上检查是否“虚假”
    std::vector<std::size_t> nn_idx(2);
    std::vector<double>      nn_dist2(2);

    std::size_t false_cnt = 0;
    std::size_t total     = 0;

    for (int i = 0; i < N_mp1; ++i) {
        // 最近邻查询：请求 k=2，通常第一个是自己，第二个是真正最近邻
        index.knn(Ym_trunc[i], 2, nn_idx, nn_dist2);

        int    chosen_j = -1;
        double d2_m     = 0.0;

        for (std::size_t t = 0; t < nn_idx.size(); ++t) {
            int j = static_cast<int>(nn_idx[t]);
            if (j == i) continue;  // 排除自身
            double d2 = nn_dist2[t];
            if (d2 <= 0.0) continue;
            chosen_j = j;
            d2_m     = d2;
            break;
        }

        if (chosen_j < 0 || d2_m <= 0.0) {
            // 没有找到有效最近邻（可能所有点都重合），不计入统计
            continue;
        }

        // 在原始时间序列上取“新维度”的值：x_{i+m*tau} 与 x_{j+m*tau}
        const int idx_i_new = i + m * tau;
        const int idx_j_new = chosen_j + m * tau;
        if (idx_i_new >= static_cast<int>(x.size()) ||
            idx_j_new >= static_cast<int>(x.size())) {
            // 安全检查：嵌入边界之外的不参与
            continue;
        }

        const double diff_new = std::fabs(x[idx_i_new] - x[idx_j_new]);
        const double d_m      = std::sqrt(d2_m);
        if (d_m <= 0.0) {
            // m 维距离为 0，说明两点在 m 维空间完全重合，此时 R 无法定义，略过
            continue;
        }

        const double R = diff_new / d_m;
        if (R > Rtol) {
            ++false_cnt;
        }
        ++total;
    }

    if (total == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(false_cnt) / static_cast<double>(total);
}

}} // namespace factorlib::math
