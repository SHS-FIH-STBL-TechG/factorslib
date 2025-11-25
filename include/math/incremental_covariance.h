// include/utils/math/incremental_covariance.h
#pragma once
/**
 * @file incremental_covariance.h
 * @brief 增量协方差计算器 - 多变量滑动窗口协方差分析
 *
 * 核心特性：
 * - 支持任意维度的多变量数据
 * - 增量计算均值向量和协方差矩阵
 * - 自动处理滑动窗口更新
 * - 数值稳定性处理
 *
 * 数学原理：
 * 协方差矩阵 Σ = E[(X-μ)(X-μ)ᵀ] = E[XXᵀ] - μμᵀ
 * 通过维护X的和与XXᵀ的和来增量计算
 *
 * 应用场景：
 * - 多因子相关性分析
 * - 风险模型协方差估计
 * - 主成分分析(PCA)预处理
 */

#include <deque>
#include <limits>
#include <Eigen/Dense>

#include "utils/log.h"

namespace factorlib {
namespace math {

/**
 * @class IncrementalCovariance
 * @brief 增量协方差计算器 - 多变量滑动窗口协方差分析
 */
template<typename T, std::size_t Dimension>
class IncrementalCovariance {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

private:
    std::size_t _window_size;  ///< 滑动窗口大小
    std::deque<Eigen::Matrix<T, Dimension, 1>> _data;  ///< 数据窗口

    // 增量统计量：使用double精度避免累积误差
    Eigen::Matrix<double, Dimension, 1> _sum;          ///< 数据向量和
    Eigen::Matrix<double, Dimension, Dimension> _sum_outer;  ///< 外积和

public:
    /**
     * @brief 构造函数
     * @param window_size 滑动窗口大小
     */
    explicit IncrementalCovariance(std::size_t window_size)
        : _window_size(window_size) {
        _sum.setZero();
        _sum_outer.setZero();
    }

    /**
     * @brief 添加新的多变量数据点
     * @param value 维度为Dimension的向量
     *
     * 处理流程：
     * 1. 数值有效性检查（NaN/Inf）
     * 2. 更新数据窗口
     * 3. 更新增量统计量
     * 4. 维护窗口大小
     */
    void push(const Eigen::Matrix<T, Dimension, 1>& value) {
        Eigen::Matrix<double, Dimension, 1> double_value = value.template cast<double>();

        // 数值有效性检查：丢弃包含NaN/Inf的数据
        if (!double_value.allFinite()) {
            LOG_DEBUG("IncrementalCovariance::push: sample has NaN/Inf, drop");
            return;
        }

        // 更新数据窗口和统计量
        _data.push_back(value);
        _sum += double_value;
        _sum_outer += double_value * double_value.transpose();

        // 维护窗口大小
        if (_data.size() > _window_size) {
            Eigen::Matrix<double, Dimension, 1> old_value =
                _data.front().template cast<double>();
            _data.pop_front();
            _sum -= old_value;
            _sum_outer -= old_value * old_value.transpose();
        }
    }

    /**
     * @brief 计算当前均值向量
     * @return 当前窗口数据的均值向量
     *
     * 公式：μ = (1/n) * Σx_i
     */
    Eigen::Matrix<double, Dimension, 1> mean() const {
        if (_data.empty())
            return Eigen::Matrix<double, Dimension, 1>::Zero();
        return _sum / _data.size();
    }

    /**
     * @brief 计算当前协方差矩阵
     * @return 当前窗口数据的协方差矩阵
     *
     * 公式：Σ = 1/(n-1) * [Σ(x_i x_iᵀ) - n * μμᵀ]
     * 注意：这是样本协方差的无偏估计版本，使用贝塞尔校正（除以n-1而不是n）
     *
     * 数学推导：
     * 无偏协方差 = 1/(n-1) * Σ(x_i - μ)(x_i - μ)ᵀ
     *            = 1/(n-1) * [Σ(x_i x_iᵀ) - n * μμᵀ]
     *
     * 其中：
     * - n: 样本数量
     * - μ: 样本均值向量
     * - x_i: 第i个样本向量
     */
    Eigen::Matrix<double, Dimension, Dimension> covariance() const {
        if (_data.size() < 2)
            return Eigen::Matrix<double, Dimension, Dimension>::Zero();

        Eigen::Matrix<double, Dimension, 1> m = mean();
        std::size_t n = _data.size();

        // 无偏协方差计算：使用贝塞尔校正（除以n-1）
        return (_sum_outer - n * m * m.transpose()) / (n - 1);
    }

    /// @brief 清空所有数据
    void clear() {
        _data.clear();
        _sum.setZero();
        _sum_outer.setZero();
    }

    /// @brief 获取当前数据数量
    std::size_t size() const { return _data.size(); }

    /// @brief 检查窗口是否已填满
    bool is_window_full() const { return _data.size() >= _window_size; }
};

} // namespace math
} // namespace factorlib