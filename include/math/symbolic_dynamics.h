#pragma once
/**
 * @file include/math/symbolic_dynamics.h
 * @brief 符号化与一阶马尔可夫转移矩阵的滑窗增量实现；
 *        支持：转移矩阵、稳态分布、熵产生率（近似）与拓扑熵（基于邻接谱半径近似）。
 */
#include <vector>
#include <deque>
#include <cmath>
#include <limits>
#include <algorithm>
#include <stdexcept>

#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

/// 简单分箱符号化：按边界切分为 k 个符号（1..k）
template<typename T=float>
class Symbolizer {
public:
    explicit Symbolizer(std::vector<double> edges) : edges_(std::move(edges)) {
        if (edges_.size() < 2) throw std::invalid_argument("edges must have >=2");
    }
    int symbol_of(T x) const {
        auto it = std::upper_bound(edges_.begin(), edges_.end(), static_cast<double>(x));
        int idx = static_cast<int>( (it == edges_.begin()) ? 0 : (it - edges_.begin() - 1) );
        if (idx >= static_cast<int>(edges_.size()-1)) idx = static_cast<int>(edges_.size()-2);
        return idx; // [0..k-1]
    }
    int k() const { return static_cast<int>(edges_.size()-1); }
private:
    std::vector<double> edges_;
};

/**
 * @tparam BadPolicy 坏值策略（默认不处理）
 * 维护一个滑窗符号序列 s_t，并对相邻对 (s_{t-1}, s_t) 累积转移计数。
 * 支持 O(1) 增量：增加一条转移，删除一条最老转移。
 */
template<typename T=float, typename BadPolicy=NoCheckBadValuePolicy>
class RollingSymbolicDynamics {
public:
    RollingSymbolicDynamics(std::size_t W, const Symbolizer<T>& sym)
        : W_(W), sym_(sym) {
        if (W_ < 3) throw std::invalid_argument("RollingSymbolicDynamics: W>=3");
        int k = sym_.k();
        counts_.assign(k, std::vector<int>(k, 0));
        row_sum_.assign(k, 0);
        buf_.assign(W_, std::numeric_limits<T>::quiet_NaN());
        symbuf_.assign(W_, -1);
    }

    bool push(T x) {
        if constexpr (std::is_floating_point_v<T>) {
            if (!is_finite_numeric(x)) {
                if (!BadPolicy::handle(x, "RollingSymbolicDynamics::push")) {
                    return false;
                }
            }
        }
        int s = sym_.symbol_of(x);
        if (size_ < W_) {
            // 预热
            buf_[tail_] = x;
            symbuf_[tail_] = s;
            if (size_ >= 1) {
                int s_prev = symbuf_[(tail_ + W_ - 1) % W_];
                add_transition(s_prev, s);
            }
            tail_ = (tail_ + 1) % W_;
            ++size_;
            return true;
        }

        // 删除最老的一个符号以及它与下一个的转移
        int s_old = symbuf_[head_];
        int s_old_next = symbuf_[(head_ + 1) % W_];
        remove_transition(s_old, s_old_next);

        // 写入新符号并添加与末尾的转移
        buf_[head_] = x;
        symbuf_[head_] = s;
        head_ = (head_ + 1) % W_;
        int s_prev = symbuf_[(head_ + W_ - 1) % W_];
        add_transition(s_prev, s);

        tail_ = (tail_ + 1) % W_;
        return true;
    }

    bool ready() const { return size_ >= W_; }

    /// 转移矩阵（行归一化）
    std::vector<std::vector<double>> transition_matrix() const {
        int k = sym_.k();
        std::vector<std::vector<double>> P(k, std::vector<double>(k, 0.0));
        for (int i = 0; i < k; ++i) {
            if (row_sum_[i] == 0) continue;
            for (int j = 0; j < k; ++j) {
                P[i][j] = static_cast<double>(counts_[i][j]) / static_cast<double>(row_sum_[i]);
            }
        }
        return P;
    }

    /// 拓扑熵近似：log(谱半径)，基于邻接矩阵 A_{ij}=1(count>0)
    double topological_entropy() const {
        int k = sym_.k();
        std::vector<std::vector<double>> A(k, std::vector<double>(k, 0.0));
        for (int i = 0; i < k; ++i) for (int j = 0; j < k; ++j)
            A[i][j] = (counts_[i][j] > 0) ? 1.0 : 0.0;
        double lambda = power_method_dominant_eig(A, 32);
        if (lambda <= 0) return std::numeric_limits<double>::quiet_NaN();
        return std::log(lambda);
    }

    /// 熵产生率（对称性近似）：sum π_i P_ij log(P_ij / P_ji)
    double entropy_production_rate() const {
        int k = sym_.k();
        // 估计稳态分布 π：采用行和的归一化（马尔可夫链的经验分布）
        long double total = 0.0L;
        for (int i = 0; i < k; ++i) total += static_cast<long double>(row_sum_[i]);
        if (total <= 0) return std::numeric_limits<double>::quiet_NaN();
        std::vector<long double> pi(k, 0.0);
        for (int i = 0; i < k; ++i) pi[i] = static_cast<long double>(row_sum_[i]) / total;

        // 行归一化的 P
        auto P = transition_matrix();
        long double sigma = 0.0L;
        for (int i = 0; i < k; ++i) for (int j = 0; j < k; ++j) {
            if (P[i][j] <= 0 || P[j][i] <= 0) continue;
            sigma += pi[i] * P[i][j] * std::log(P[i][j] / P[j][i]);
        }
        return static_cast<double>(sigma);
    }

    /// 转移矩阵的主特征值（谱半径）近似
    double leading_eigenvalue() const {
        auto P = transition_matrix();
        return power_method_dominant_eig(P, 32);
    }

private:
    void add_transition(int i, int j) {
        ++counts_[i][j];
        ++row_sum_[i];
    }
    void remove_transition(int i, int j) {
        if (counts_[i][j] > 0) { --counts_[i][j]; }
        if (row_sum_[i] > 0) { --row_sum_[i]; }
    }
    static double power_method_dominant_eig(const std::vector<std::vector<double>>& A, int iters) {
        int n = static_cast<int>(A.size());
        std::vector<double> v(n, 1.0 / n);
        for (int it = 0; it < iters; ++it) {
            std::vector<double> Av(n, 0.0);
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) Av[i] += A[i][j] * v[j];
            }
            double norm = 0.0; for (double x: Av) norm += x*x; norm = std::sqrt(norm);
            if (norm == 0) return 0.0;
            for (int i = 0; i < n; ++i) v[i] = Av[i] / norm;
        }
        // Rayleigh quotient 近似特征值
        std::vector<double> Av(n, 0.0);
        for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) Av[i] += A[i][j] * v[j];
        double num = 0.0, den = 0.0;
        for (int i = 0; i < n; ++i) { num += v[i]*Av[i]; den += v[i]*v[i]; }
        return (den == 0) ? 0.0 : (num/den);
    }

private:
    std::size_t W_;
    Symbolizer<T> sym_;
    std::vector<T> buf_;
    std::vector<int> symbuf_;
    std::size_t head_{0}, tail_{0}, size_{0};
    std::vector<std::vector<int>> counts_;
    std::vector<int> row_sum_;
};

}} // namespace factorlib::math
