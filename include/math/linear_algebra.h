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
 * 提供常用的线性代数运算，包括协方差矩阵计算、条件数计算、矩阵正则化等
 * @tparam T 数值类型，必须是算术类型
 */
template<typename T>
class LinearAlgebra {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

public:
    /**
     * @brief 计算数据集的协方差矩阵
     * @tparam OuterContainer 外层容器类型，包含内层数据容器
     * @param data 输入数据，外层容器包含多个内层容器，每个内层容器代表一个数据点
     * @return Eigen::MatrixXd 计算得到的协方差矩阵
     * @note 数据会自动转换为double类型进行计算
     */
    template<typename OuterContainer>
    static Eigen::MatrixXd covariance_matrix(const OuterContainer& data) {
        if (data.empty()) {
            return Eigen::MatrixXd(0, 0);
        }

        // 获取数据维度
        const size_t n_samples = data.size();
        const size_t n_features = data.begin()->size();

        // 检查数据一致性
        for (const auto& sample : data) {
            if (sample.size() != n_features) {
                throw std::invalid_argument("All data samples must have the same dimension");
            }
        }

        // 单一样本情况：无法计算协方差，返回零矩阵
        if (n_samples == 1) {
            return Eigen::MatrixXd::Zero(n_features, n_features);
        }

        // 转换为Eigen矩阵
        Eigen::MatrixXd mat(n_samples, n_features);
        size_t row_idx = 0;
        for (const auto& sample : data) {
            size_t col_idx = 0;
            for (const auto& value : sample) {
                mat(row_idx, col_idx) = static_cast<double>(value);
                ++col_idx;
            }
            ++row_idx;
        }

        // 计算列均值
        const Eigen::VectorXd mean = mat.colwise().mean();

        // 中心化矩阵：每个样本减去均值
        const Eigen::MatrixXd centered = mat.rowwise() - mean.transpose();

        // 计算协方差矩阵: (X^T * X) / (n-1)
        return (centered.adjoint() * centered) / static_cast<double>(n_samples - 1);
    }

    /**
     * @brief 计算矩阵的条件数（2-范数条件数）
     * @param matrix 输入矩阵
     * @return double 条件数值，无穷大表示矩阵是奇异的
     * @note 条件数 = 最大奇异值 / 最小奇异值，反映矩阵求逆的稳定性
     */
    static double condition_number(const Eigen::MatrixXd& matrix) {
        if (matrix.rows() == 0 || matrix.cols() == 0) {
            return 0.0;
        }

        // 使用SVD分解计算奇异值
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
        const auto& singular_values = svd.singularValues();

        // 检查最小奇异值是否为0（矩阵奇异）
        if (singular_values.minCoeff() <= std::numeric_limits<double>::epsilon()) {
            return std::numeric_limits<double>::infinity();
        }

        return singular_values.maxCoeff() / singular_values.minCoeff();
    }

    /**
     * @brief 正则化协方差矩阵，提高数值稳定性
     * @param cov_matrix 输入的协方差矩阵
     * @param regularization 正则化参数，默认为1e-6
     * @return Eigen::MatrixXd 正则化后的协方差矩阵
     * @note 通过对角线添加小值来改善矩阵的条件数
     */
    static Eigen::MatrixXd regularize_covariance(const Eigen::MatrixXd& cov_matrix,
                                                double regularization = 1e-6) {
        if (cov_matrix.rows() != cov_matrix.cols()) {
            throw std::invalid_argument("Covariance matrix must be square");
        }

        return cov_matrix + Eigen::MatrixXd::Identity(cov_matrix.rows(),
                                                     cov_matrix.cols()) * regularization;
    }

    /**
     * @brief 计算多元正态分布的条件期望
     * 给定部分变量的观测值，计算目标变量的条件期望
     * @param mean 多元正态分布的均值向量
     * @param covariance 多元正态分布的协方差矩阵
     * @param condition_values 条件变量的观测值向量
     * @param target_index 目标变量在均值向量中的索引
     * @return double 目标变量的条件期望值
     * @note 使用公式: E[X|Y] = μ_x + Σ_xy * Σ_yy^{-1} * (Y - μ_y)
     */
    static double conditional_expectation(const Eigen::VectorXd& mean,
                                         const Eigen::MatrixXd& covariance,
                                         const Eigen::VectorXd& condition_values,
                                         size_t target_index) {
        // 参数检查
        if (covariance.rows() != covariance.cols()) {
            throw std::invalid_argument("Covariance matrix must be square");
        }
        if (mean.size() != covariance.rows()) {
            throw std::invalid_argument("Mean vector dimension must match covariance matrix");
        }
        if (condition_values.size() != mean.size() - 1) {
            throw std::invalid_argument("Condition values dimension must be mean dimension - 1");
        }
        if (target_index >= mean.size()) {
            throw std::invalid_argument("Target index out of range");
        }

        const size_t n_total = mean.size();
        const size_t n_condition = condition_values.size();

        // 重新排列变量顺序：将目标变量放在最后
        Eigen::VectorXd reordered_mean(n_total);
        Eigen::MatrixXd reordered_cov(n_total, n_total);

        // 构建条件变量的索引（排除目标变量）
        std::vector<size_t> condition_indices;
        for (size_t i = 0; i < n_total; ++i) {
            if (i != target_index) {
                condition_indices.push_back(i);
            }
        }

        // 重新排列均值向量：[条件变量..., 目标变量]
        for (size_t i = 0; i < n_condition; ++i) {
            reordered_mean(i) = mean(condition_indices[i]);
        }
        reordered_mean(n_condition) = mean(target_index);

        // 重新排列协方差矩阵
        for (size_t i = 0; i < n_condition; ++i) {
            for (size_t j = 0; j < n_condition; ++j) {
                reordered_cov(i, j) = covariance(condition_indices[i], condition_indices[j]);
            }
            reordered_cov(i, n_condition) = covariance(condition_indices[i], target_index);
            reordered_cov(n_condition, i) = covariance(target_index, condition_indices[i]);
        }
        reordered_cov(n_condition, n_condition) = covariance(target_index, target_index);

        // 分割协方差矩阵
        // Σ = [[Σ_11, Σ_12],  其中 Σ_11: 条件变量协方差, Σ_22: 目标变量方差
        //      [Σ_21, Σ_22]]       Σ_12/Σ_21: 条件变量与目标变量的协方差
        const Eigen::MatrixXd sigma_11 = reordered_cov.block(0, 0, n_condition, n_condition);
        const Eigen::VectorXd sigma_12 = reordered_cov.block(0, n_condition, n_condition, 1);
        const double sigma_22 = reordered_cov(n_condition, n_condition);

        // 分割均值向量
        const Eigen::VectorXd mean_1 = reordered_mean.head(n_condition);
        const double mean_2 = reordered_mean(n_condition);

        // 计算条件期望: μ_2|1 = μ_2 + Σ_21 * Σ_11^{-1} * (x_1 - μ_1)
        const Eigen::VectorXd x_condition_diff = condition_values - mean_1;

        // 使用LDLT分解求解线性系统，比直接求逆更稳定
        const Eigen::VectorXd beta = sigma_11.ldlt().solve(sigma_12);

        return mean_2 + beta.dot(x_condition_diff);
    }
};

} // namespace math
} // namespace factorlib