#pragma once
/**
 * @file include/math/online_pca.h
 * @brief 增量 PCA（Oja 法 + 流式均值/协方差），适合高频维度 <= 16 的场景。
 *
 * 我们提供两个接口：
 *  - push(x): 逐条样本增量更新均值与主方向（前 k 个向量），避免全量重算；
 *  - components(): 返回当前主成分向量（单位化），与 explained_variance()。
 *
 * 复杂度：每步 O(d*k)，d 为维度，k 为主成分数。
 */
#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

template<typename T=float, typename BadPolicy=NoCheckBadValuePolicy>
class OnlinePCA {
public:
    OnlinePCA(int d, int k, double lr=0.05)
        : d_(d), k_(k), lr_(lr) {
        if (d_ <= 0 || k_ <= 0 || k_ > d_) throw std::invalid_argument("OnlinePCA: invalid d/k");
        mean_.assign(d_, 0.0);
        comps_.assign(k_, std::vector<double>(d_, 0.0));
        // 初始化为单位基向量
        for (int i = 0; i < k_; ++i) if (i < d_) comps_[i][i] = 1.0;
        var_sum_ = 0.0; n_ = 0;
    }

    bool push(const std::vector<T>& x) {
        if ((int)x.size() != d_) throw std::invalid_argument("OnlinePCA: size mismatch");
        // 坏值检查：遇到 NaN 则按策略处理（默认直接使用，或调用方预先清洗）
        for (int i = 0; i < d_; ++i) {
            if constexpr (std::is_floating_point_v<T>) {
                if (!is_finite_numeric(x[i])) {
                    T tmp = x[i];
                    if (!BadPolicy::handle(tmp, "OnlinePCA::push")) return false;
                }
            }
        }
        // 更新均值（Welford）
        ++n_;
        double rn = static_cast<double>(n_);
        for (int i = 0; i < d_; ++i) {
            double delta = static_cast<double>(x[i]) - mean_[i];
            mean_[i] += delta / rn;
        }
        // 中心化向量
        std::vector<double> xc(d_);
        for (int i = 0; i < d_; ++i) xc[i] = static_cast<double>(x[i]) - mean_[i];

        // Oja 更新：依次更新每个主向量，并做 Gram-Schmidt 正交化
        for (int j = 0; j < k_; ++j) {
            double proj = dot(xc, comps_[j]);
            for (int i = 0; i < d_; ++i) comps_[j][i] += lr_ * proj * (xc[i] - proj * comps_[j][i]);
            // 归一化
            normalize(comps_[j]);
            // 从 xc 中移除该主方向的分量（近似的降秩处理）
            for (int i = 0; i < d_; ++i) xc[i] -= proj * comps_[j][i];
        }

        // 更新总方差累积（用于解释方差比估计）
        double sq = 0.0; for (int i = 0; i < d_; ++i) { double xi = static_cast<double>(x[i]); sq += (xi - mean_[i])*(xi - mean_[i]); }
        var_sum_ += sq;
        return true;
    }

    const std::vector<std::vector<double>>& components() const { return comps_; }

    /// 近似的解释方差（用投影方差累积来估计每个分量）
    std::vector<double> explained_variance(const std::vector<std::vector<T>>& recent) const {
        std::vector<double> var(k_, 0.0);
        if (recent.empty()) return var;
        // 计算在最近一批样本上的投影方差
        for (const auto& x : recent) {
            std::vector<double> xc(d_);
            for (int i = 0; i < d_; ++i) xc[i] = static_cast<double>(x[i]) - mean_[i];
            for (int j = 0; j < k_; ++j) {
                double proj = dot(xc, comps_[j]);
                var[j] += proj * proj;
            }
        }
        for (int j = 0; j < k_; ++j) var[j] /= static_cast<double>(recent.size());
        return var;
    }

private:
    static double dot(const std::vector<double>& a, const std::vector<double>& b) {
        double s = 0.0; for (size_t i = 0; i < a.size(); ++i) s += a[i]*b[i]; return s;
    }
    static void normalize(std::vector<double>& v) {
        double n = 0.0; for (double x : v) n += x*x; n = std::sqrt(n);
        if (n > 0) for (double& x : v) x /= n;
    }

private:
    int d_, k_;
    double lr_;
    std::vector<double> mean_;
    std::vector<std::vector<double>> comps_;
    double var_sum_;
    long long n_;
};

}} // namespace factorlib::math
