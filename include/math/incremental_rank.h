// include/utils/math/incremental_rank.h
#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <set>
#include <type_traits>
#include <vector>
#include <Eigen/Dense>

#include "utils/log.h"
#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

/**
 * @brief 增量秩计算器 - 使用平衡二叉搜索树维护排序，实现 O(log n) 的插入和删除
 */
template<typename T, typename BadValuePolicy = NoCheckBadValuePolicy>
class IncrementalRankCalculator {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

private:
    std::multiset<T> _sorted_data;
    std::deque<T> _window_data;  // 维护插入顺序，用于删除最旧元素

public:
    /// 添加新数据点（自动维护窗口大小）
    void push(T value, size_t window_size) {
        // 统一入口：NaN/Inf 先按策略处理
        if (!BadValuePolicy::handle(value, "IncrementalRankCalculator::push")) {
            return;  // 丢弃这一条
        }

        _sorted_data.insert(value);
        _window_data.push_back(value);

        // 如果超过窗口大小，移除最旧的数据
        if (_window_data.size() > window_size) {
            T oldest_value = _window_data.front();
            _window_data.pop_front();

            // 从排序集合中移除最旧的值
            auto it = _sorted_data.find(oldest_value);
            if (it != _sorted_data.end()) {
                _sorted_data.erase(it);
            }
        }
    }

    /// 计算中位秩
    double median_rank(T value) const {
        if (_sorted_data.empty()) return 0.5;

        // 找到值在排序数据中的位置
        auto it = _sorted_data.lower_bound(value);
        size_t rank = std::distance(_sorted_data.begin(), it);

        return (rank + 0.5) / _sorted_data.size();
    }

    /// 获取排序后的数据（用于经验逆CDF计算）
    std::vector<T> get_sorted_data() const {
        return std::vector<T>(_sorted_data.begin(), _sorted_data.end());
    }

    /// 获取当前数据大小
    size_t size() const {
        return _sorted_data.size();
    }

    /// 清空所有数据
    void clear() {
        _sorted_data.clear();
        _window_data.clear();
    }

    /// 检查是否已达到窗口大小
    bool is_window_full(size_t window_size) const {
        return _sorted_data.size() >= window_size;
    }
};

/**
 * @brief 增量协方差计算器 - 支持多变量滑动窗口
 */
template<typename T, size_t Dimension>
class IncrementalCovariance {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

private:
    size_t _window_size;
    std::deque<Eigen::Matrix<T, Dimension, 1>> _data;

    // 增量统计量
    Eigen::Matrix<double, Dimension, 1> _sum;
    Eigen::Matrix<double, Dimension, Dimension> _sum_outer;

public:
    explicit IncrementalCovariance(size_t window_size)
        : _window_size(window_size) {
        _sum.setZero();
        _sum_outer.setZero();
    }

    /// 添加新的多变量数据点
    void push(const Eigen::Matrix<T, Dimension, 1>& value) {
        Eigen::Matrix<double, Dimension, 1> double_value = value.template cast<double>();

        // 统一检查：如果样本中有 NaN/Inf，直接丢弃并记日志
        if (!double_value.allFinite()) {
            LOG_WARN("IncrementalCovariance::push: sample has NaN/Inf, drop");
            return;
        }

        _data.push_back(value);
        _sum += double_value;
        _sum_outer += double_value * double_value.transpose();

        if (_data.size() > _window_size) {
            Eigen::Matrix<double, Dimension, 1> old_value = _data.front().template cast<double>();
            _data.pop_front();
            _sum -= old_value;
            _sum_outer -= old_value * old_value.transpose();
        }
    }

    /// 获取当前均值向量（增量计算）
    Eigen::Matrix<double, Dimension, 1> mean() const {
        if (_data.empty()) return Eigen::Matrix<double, Dimension, 1>::Zero();
        return _sum / _data.size();
    }

    /// 获取当前协方差矩阵（增量计算）
    Eigen::Matrix<double, Dimension, Dimension> covariance() const {
        if (_data.size() < 2) return Eigen::Matrix<double, Dimension, Dimension>::Zero();

        Eigen::Matrix<double, Dimension, 1> m = mean();
        return (_sum_outer / _data.size()) - (m * m.transpose());
    }

    /// 清空所有数据
    void clear() {
        _data.clear();
        _sum.setZero();
        _sum_outer.setZero();
    }

    size_t size() const {
        return _data.size();
    }

    /// 检查是否已达到窗口大小
    bool is_window_full() const {
        return _data.size() >= _window_size;
    }
};

} // namespace math
} // namespace factorlib