// include/utils/math/linear_algebra.h
#pragma once
#include <vector>
#include <deque>
#include <type_traits>
#include <Eigen/Dense>

namespace factorlib {
namespace math {

/**
 * @brief 线性代数工具模板类
 */
template<typename T>
class LinearAlgebra {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

public:
    /// 计算矩阵的协方差矩阵 - 支持任意容器类型
    template<typename OuterContainer>
    static Eigen::MatrixXd covariance_matrix(const OuterContainer& data) {
        if (data.empty()) return Eigen::MatrixXd();

        size_t n = data.size();
        size_t m = data.begin()->size();

        // 转换为Eigen矩阵
        Eigen::MatrixXd mat(n, m);
        size_t i = 0;
        for (const auto& inner_container : data) {
            size_t j = 0;
            for (const auto& value : inner_container) {
                mat(i, j) = static_cast<double>(value);
                ++j;
            }
            ++i;
        }

        // 计算均值
        Eigen::VectorXd mean = mat.colwise().mean();

        // 中心化矩阵
        Eigen::MatrixXd centered = mat.rowwise() - mean.transpose();

        // 计算协方差矩阵
        return (centered.adjoint() * centered) / double(n - 1);
    }

    /// 计算矩阵的条件数
    static double condition_number(const Eigen::MatrixXd& matrix) {
        if (matrix.rows() == 0 || matrix.cols() == 0) return 0.0;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
        auto singular_values = svd.singularValues();

        if (singular_values.minCoeff() == 0) {
            return std::numeric_limits<double>::infinity();
        }

        return singular_values.maxCoeff() / singular_values.minCoeff();
    }

    /// 正则化协方差矩阵
    static Eigen::MatrixXd regularize_covariance(const Eigen::MatrixXd& cov_matrix,
                                                double regularization = 1e-6) {
        return cov_matrix + Eigen::MatrixXd::Identity(cov_matrix.rows(),
                                                     cov_matrix.cols()) * regularization;
    }

    /// 计算多元正态分布的条件期望
    static double conditional_expectation(const Eigen::VectorXd& mean,
                                         const Eigen::MatrixXd& covariance,
                                         const Eigen::VectorXd& condition_values,
                                         size_t target_index) {
        if (covariance.rows() != covariance.cols() ||
            mean.size() != covariance.rows() ||
            condition_values.size() != mean.size() - 1) {
            return 0.0;
        }

        // 分割协方差矩阵
        // Σ = [[Σ_11, Σ_12],
        //      [Σ_21, Σ_22]]
        // 其中 Σ_11 是条件变量的协方差，Σ_22 是目标变量的方差

        size_t n_condition = condition_values.size();
        Eigen::MatrixXd sigma_11 = covariance.block(0, 0, n_condition, n_condition);
        Eigen::VectorXd sigma_12 = covariance.block(0, n_condition, n_condition, 1);
        double sigma_22 = covariance(n_condition, n_condition);

        // 分割均值向量
        Eigen::VectorXd mean_1 = mean.head(n_condition);
        double mean_2 = mean(n_condition);

        // 条件均值公式: μ_2|1 = μ_2 + Σ_21 * Σ_11^{-1} * (x_1 - μ_1)
        Eigen::VectorXd x_condition = condition_values - mean_1;
        Eigen::VectorXd beta = sigma_11.ldlt().solve(sigma_12);

        return mean_2 + beta.dot(x_condition);
    }
};

} // namespace math
} // namespace factorlib