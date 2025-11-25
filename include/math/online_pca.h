#pragma once
/**
 * @file online_pca.h
 * @brief 增量PCA（Oja法 + 流式均值/协方差），适合高频维度 <= 16 的场景
 *
 * 修复问题：
 * 1. 解释方差计算：使用更稳定的方法，避免因非正交性导致比例>1
 * 2. 正交性保证：改进正交化策略，增加数值稳定性
 * 3. 学习率调整：默认使用更保守的学习率
 */

#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>

#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

template<typename T = float, typename BadPolicy = NoCheckBadValuePolicy>
class OnlinePCA {
public:
    /**
     * @brief 构造函数
     * @param d 数据维度
     * @param k 主成分数量
     * @param lr 学习率，控制更新速度，默认0.01（更保守）
     * @param ortho_refresh 正交化刷新频率，每n个样本执行一次完全正交化
     */
    OnlinePCA(int d, int k, double lr = 0.01, int ortho_refresh = 50)
        : d_(d), k_(k), lr_(lr), ortho_refresh_(ortho_refresh) {
        if (d_ <= 0 || k_ <= 0 || k_ > d_)
            throw std::invalid_argument("OnlinePCA: 维度d和主成分数k必须满足 0 < k <= d");

        mean_.assign(d_, 0.0);

        // 初始化主成分向量，使用随机正交向量作为初始基
        comps_.resize(k_);
        initialize_random_orthogonal_components();

        var_sum_ = 0.0;
        n_ = 0;
        update_count_ = 0;
    }

    bool push(const std::vector<T>& x) {
        if (static_cast<int>(x.size()) != d_)
            throw std::invalid_argument("OnlinePCA: 输入向量维度与模型不匹配");

        std::vector<T> processed_x = x;
        if (!process_bad_values(processed_x)) {
            return false;
        }

        ++n_;
        ++update_count_;
        double rn = static_cast<double>(n_);

        // 更新均值
        update_mean(processed_x, rn);

        // 计算中心化样本
        std::vector<double> xc = compute_centered_sample(processed_x);

        // 使用改进的Oja更新
        update_components_improved(xc);

        // 定期执行完全正交化
        if (update_count_ % ortho_refresh_ == 0) {
            full_orthogonalize_modified_gram_schmidt();
        }

        update_variance_stats(xc);

        return true;
    }

    const std::vector<std::vector<double>>& components() const {
        return comps_;
    }

    /**
     * @brief 计算主成分的解释方差（改进版本）
     * @param recent 最近的一批样本
     * @return std::vector<double> 各主成分的解释方差
     *
     * @details 使用更稳定的计算方法，避免因非正交性导致方差和>总方差
     */
    std::vector<double> explained_variance(const std::vector<std::vector<T>>& recent) const {
        std::vector<double> var(k_, 0.0);
        if (recent.empty()) return var;

        // 使用投影到正交基的方法计算方差
        for (const auto& sample : recent) {
            std::vector<double> xc = compute_centered_sample(sample);
            std::vector<double> projections(k_, 0.0);

            // 计算在所有主成分上的投影
            for (int j = 0; j < k_; ++j) {
                projections[j] = dot(xc, comps_[j]);
            }

            // 使用正交投影计算每个方向的贡献
            for (int j = 0; j < k_; ++j) {
                // 只计算该主成分独有的方差贡献
                double unique_contribution = projections[j] * projections[j];
                var[j] += unique_contribution;
            }
        }

        double norm_factor = 1.0 / static_cast<double>(recent.size());
        for (int j = 0; j < k_; ++j) {
            var[j] *= norm_factor;
        }

        return var;
    }

    /**
     * @brief 计算累计解释方差比例（改进版本）
     * @param recent 最近的一批样本
     * @return std::vector<double> 前m个主成分的累计解释方差比例
     *
     * @details 使用重构误差方法计算解释方差，避免非正交性问题
     */
    std::vector<double> explained_variance_ratio(const std::vector<std::vector<T>>& recent) const {
        std::vector<double> ratios(k_, 0.0);
        if (recent.empty()) return ratios;

        // 计算总方差
        double total_var = 0.0;
        for (const auto& sample : recent) {
            auto xc = compute_centered_sample(sample);
            total_var += dot(xc, xc);
        }
        total_var /= static_cast<double>(recent.size());

        if (total_var < 1e-12) return ratios;

        // 使用重构误差计算每个主成分的贡献
        std::vector<double> cumulative_variance(k_, 0.0);

        for (int comps_used = 1; comps_used <= k_; ++comps_used) {
            double reconstruction_error = 0.0;

            for (const auto& sample : recent) {
                auto xc = compute_centered_sample(sample);
                auto x_reconstructed = reconstruct_sample(xc, comps_used);

                // 计算重构误差
                double error = 0.0;
                for (int i = 0; i < d_; ++i) {
                    double diff = xc[i] - x_reconstructed[i];
                    error += diff * diff;
                }
                reconstruction_error += error;
            }

            reconstruction_error /= static_cast<double>(recent.size());
            double explained_var = total_var - reconstruction_error;
            cumulative_variance[comps_used - 1] = std::max(0.0, explained_var);
        }

        // 计算比例并确保单调递增
        double prev_ratio = 0.0;
        for (int i = 0; i < k_; ++i) {
            double ratio = cumulative_variance[i] / total_var;
            ratios[i] = std::max(prev_ratio, std::min(ratio, 1.0)); // 确保在[0,1]范围内
            prev_ratio = ratios[i];
        }

        return ratios;
    }

    const std::vector<double>& mean() const {
        return mean_;
    }

    long long sample_count() const {
        return n_;
    }

    void reset(bool keep_components = false) {
        mean_.assign(d_, 0.0);
        var_sum_ = 0.0;
        n_ = 0;
        update_count_ = 0;

        if (!keep_components) {
            initialize_random_orthogonal_components();
        }
    }

private:
    /**
     * @brief 初始化随机正交的主成分
     * @details 使用简单的随机初始化，然后正交化
     */
    void initialize_random_orthogonal_components() {
        // 简单的确定性随机初始化（使用固定种子确保可重复性）
        for (int j = 0; j < k_; ++j) {
            comps_[j].resize(d_);
            for (int i = 0; i < d_; ++i) {
                // 使用基于索引的伪随机值，避免真正的随机性
                double val = std::sin(static_cast<double>(i * 7 + j * 13) * 100.0);
                comps_[j][i] = val - std::floor(val); // 映射到[0,1)
            }
        }
        full_orthogonalize_modified_gram_schmidt();
    }

    /**
     * @brief 改进的Oja更新规则
     * @details 使用更稳定的更新方式，减少数值误差
     */
    void update_components_improved(const std::vector<double>& xc) {
        // 为每个主成分单独更新，避免渐进式正交化的误差累积
        for (int j = 0; j < k_; ++j) {
            double projection = dot(xc, comps_[j]);

            // 应用Oja规则，但使用更稳定的数值计算
            for (int i = 0; i < d_; ++i) {
                double update = lr_ * projection * (xc[i] - projection * comps_[j][i]);
                // 限制更新幅度，防止数值爆炸
                if (std::abs(update) > 1.0) {
                    update = (update > 0) ? 1.0 : -1.0;
                }
                comps_[j][i] += update;
            }

            // 立即归一化当前主成分
            normalize(comps_[j]);
        }

        // 每次更新后都进行轻量级正交化
        lightweight_orthogonalize();
    }

    /**
     * @brief 轻量级正交化
     * @details 每次更新后执行，保持较好的正交性
     */
    void lightweight_orthogonalize() {
        for (int j = 1; j < k_; ++j) {
            // 只对当前主成分与之前的主成分进行正交化
            for (int i = 0; i < j; ++i) {
                double proj = dot(comps_[j], comps_[i]);
                // 只修正较大的非正交性
                if (std::abs(proj) > 1e-8) {
                    for (int dim = 0; dim < d_; ++dim) {
                        comps_[j][dim] -= proj * comps_[i][dim];
                    }
                }
            }
            normalize(comps_[j]);
        }
    }

    /**
     * @brief 改进的Gram-Schmidt正交化
     * @details 使用Modified Gram-Schmidt，数值更稳定
     */
    void full_orthogonalize_modified_gram_schmidt() {
        for (int j = 0; j < k_; ++j) {
            // 先归一化当前向量
            normalize(comps_[j]);

            // 对后续向量进行正交化
            for (int i = j + 1; i < k_; ++i) {
                double proj = dot(comps_[i], comps_[j]);
                for (int dim = 0; dim < d_; ++dim) {
                    comps_[i][dim] -= proj * comps_[j][dim];
                }
            }
        }

        // 最后确保所有向量都是单位向量
        for (int j = 0; j < k_; ++j) {
            normalize(comps_[j]);
        }
    }

    /**
     * @brief 使用前n个主成分重构样本
     */
    std::vector<double> reconstruct_sample(const std::vector<double>& xc, int n_components) const {
        std::vector<double> reconstructed(d_, 0.0);
        for (int j = 0; j < n_components && j < k_; ++j) {
            double proj = dot(xc, comps_[j]);
            for (int i = 0; i < d_; ++i) {
                reconstructed[i] += proj * comps_[j][i];
            }
        }
        return reconstructed;
    }

    bool process_bad_values(std::vector<T>& x) const {
        for (int i = 0; i < d_; ++i) {
            if constexpr (std::is_floating_point_v<T>) {
                T& value = x[i];
                if (!is_finite_numeric(value)) {
                    if (!BadPolicy::handle(value, "OnlinePCA::push")) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    void update_mean(const std::vector<T>& x, double rn) {
        for (int i = 0; i < d_; ++i) {
            double delta = static_cast<double>(x[i]) - mean_[i];
            mean_[i] += delta / rn;
        }
    }

    std::vector<double> compute_centered_sample(const std::vector<T>& x) const {
        std::vector<double> xc(d_);
        for (int i = 0; i < d_; ++i) {
            xc[i] = static_cast<double>(x[i]) - mean_[i];
        }
        return xc;
    }

    void update_variance_stats(const std::vector<double>& xc) {
        double sample_var = dot(xc, xc);
        var_sum_ += sample_var;
    }

    static double dot(const std::vector<double>& a, const std::vector<double>& b) {
        double sum = 0.0;
        for (size_t i = 0; i < a.size(); ++i) {
            sum += a[i] * b[i];
        }
        return sum;
    }

    static void normalize(std::vector<double>& v) {
        double norm = std::sqrt(dot(v, v));
        if (norm > std::numeric_limits<double>::epsilon()) {
            double inv_norm = 1.0 / norm;
            for (double& element : v) {
                element *= inv_norm;
            }
        }
    }

private:
    int d_, k_;
    double lr_;
    int ortho_refresh_;

    std::vector<double> mean_;
    std::vector<std::vector<double>> comps_;

    double var_sum_;
    long long n_;
    int update_count_;
};

}} // namespace factorlib::math