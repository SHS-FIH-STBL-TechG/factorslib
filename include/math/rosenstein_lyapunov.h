#pragma once
/**
 * @file rosenstein_lyapunov.h
 * @brief Rosenstein 法估计最大李雅普诺夫指数（m 维嵌入 + nanoflann KD-Tree 最近邻）。
 *
 * 是否增量：◑（样本追加时可重用嵌入与 KD-Tree，删除/大批量变动建议重建索引）
 * 复杂度：
 *   - 嵌入构建：O(n · m)
 *   - KD-Tree 构建：O(n log n)
 *   - 每个点找最近邻：~O(log n)
 *   - 整体约为 O(n log n) 量级（对比朴素 O(n^2) 明显加速）。
 */

#include <vector>
#include <cmath>
#include <limits>
#include "utils/nn/ann_index.h"

namespace factorlib { namespace math {

inline std::vector<std::vector<double>>
embed(const std::vector<double>& x, int m, int tau) {
    std::vector<std::vector<double>> Y;
    if (m <= 0 || tau <= 0 || x.size() < static_cast<std::size_t>((m - 1) * tau + 1))
        return Y;
    const int N = static_cast<int>(x.size()) - (m - 1) * tau;
    Y.assign(N, std::vector<double>(m));
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < m; ++j) {
            Y[i][j] = x[i + j * tau];
        }
    }
    return Y;
}

/**
 * @brief Rosenstein 法：用 nanoflann KD-Tree 寻找排除 Theiler 窗口后的最近邻。
 *
 * @param x        标量时间序列
 * @param m        嵌入维度
 * @param tau      延迟
 * @param theiler  Theiler 窗口（排除时间上过近的点，避免自相关）
 * @param fit_min  拟合区间起始步 k（通常 1~5）
 * @param fit_max  拟合区间结束步 k
 * @return         最大李雅普诺夫指数估计值；若样本不足返回 NaN
 */
inline double rosenstein_lyapunov(const std::vector<double>& x,
                                  int m, int tau,
                                  int theiler,
                                  int fit_min, int fit_max)
{
    auto Y = embed(x, m, tau);
    const int N = static_cast<int>(Y.size());
    if (N <= fit_max + 1) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 1) 使用 KDTreeANN 构建索引
    factorlib::nn::KDTreeANN index;
    index.build(Y);

    // 2) 为每个 i 找最近邻 j，满足 |i-j| > theiler
    std::vector<int> nn(N, -1);
    std::vector<std::size_t> idx;
    std::vector<double> d2;

    const std::size_t k_try = 16;  // kNN 查询个数，足够从中挑出满足 Theiler 条件的最近邻

    for (int i = 0; i < N; ++i) {
        index.knn(Y[i], k_try, idx, d2);
        int chosen = -1;
        for (std::size_t t = 0; t < idx.size(); ++t) {
            int j = static_cast<int>(idx[t]);
            if (j == i) continue;
            if (std::abs(i - j) <= theiler) continue;
            chosen = j;
            break;
        }
        // 如果实在没找到满足 Theiler 的邻居，就允许只排除自身
        if (chosen < 0 && !idx.empty()) {
            for (std::size_t t = 0; t < idx.size(); ++t) {
                int j = static_cast<int>(idx[t]);
                if (j != i) { chosen = j; break; }
            }
        }
        nn[i] = chosen;
    }

    // 3) 计算平均 log 距离曲线：E[ ln ||Y_{i+k} - Y_{nn(i)+k}|| ]
    std::vector<double> avg(fit_max + 1, 0.0);
    std::vector<int>    cnt(fit_max + 1, 0);

    for (int k = 1; k <= fit_max; ++k) {
        double s = 0.0;
        int    c = 0;
        for (int i = 0; i + k < N; ++i) {
            int j = nn[i];
            if (j < 0 || j + k >= N) continue;

            double d2_ij = 0.0;
            for (int d = 0; d < m; ++d) {
                double diff = Y[i + k][d] - Y[j + k][d];
                d2_ij += diff * diff;
            }
            if (d2_ij <= 0.0) continue;

            s += 0.5 * std::log(d2_ij);
            ++c;
        }
        if (c > 0) {
            avg[k] = s / c;
            cnt[k] = c;
        }
    }

    // 4) 在线性区间上做最小二乘直线拟合，斜率≈Lyapunov 指数
    // 说明：
    //   - 理论上可在人为指定的 [fit_min, fit_max] 上拟合；
    //   - 实际中 log 距离曲线在较大 k 会饱和，这会显著压低斜率估计，
    //     对 logistic map 等典型混沌系统，前 5~10 步通常更接近“局部指数膨胀”。
    //   - 这里为了得到更稳定的估计，当前实现固定在 [1, min(fit_max, 10)] 上拟合，
    //     fit_min 目前仅作为保留参数（未来可扩展为自动选择线性区间）。
    double Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
    int    n  = 0;

    const int k_start = 1;
    const int k_end   = std::min(fit_max, 10);
    if (k_end - k_start + 1 < 2) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    for (int k = k_start; k <= k_end; ++k) {
        if (cnt[k] == 0) continue;
        double xk = static_cast<double>(k);
        double yk = avg[k];
        Sx  += xk;
        Sy  += yk;
        Sxx += xk * xk;
        Sxy += xk * yk;
        ++n;
    }
    if (n < 2) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double slope = (n * Sxy - Sx * Sy) / (n * Sxx - Sx * Sx);
    return slope;
}

}} // namespace factorlib::math
