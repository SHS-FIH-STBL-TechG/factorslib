// include/utils/math/statistics.h
#pragma once
#include <vector>
#include <deque>
#include <algorithm>
#include <numeric>
#include <type_traits>
#include <cmath>
#include <iterator>

namespace factorlib {
namespace math {

/**
 * @brief 统计计算模板类 - 支持任意数值类型和容器类型
 */
template<typename T>
class Statistics {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

public:
    /// 计算均值 - 支持任意容器类型
    template<typename Container>
    static double mean(const Container& data) {
        if (data.empty()) return 0.0;
        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        return sum / data.size();
    }

    /// 计算标准差 - 支持任意容器类型
    template<typename Container>
    static double stddev(const Container& data) {
        if (data.size() < 2) return 0.0;
        double m = mean(data);
        double sum_sq = 0.0;
        for (const auto& x : data) {
            sum_sq += (x - m) * (x - m);
        }
        return std::sqrt(sum_sq / (data.size() - 1));
    }

    /// 计算分位数 - 支持任意容器类型
    template<typename Container>
    static double quantile(const Container& data, double percentile) {
        if (data.empty()) return 0.0;

        // 创建排序副本
        std::vector<T> sorted_data(data.begin(), data.end());
        std::sort(sorted_data.begin(), sorted_data.end());

        double position = percentile * (sorted_data.size() - 1);
        size_t index = static_cast<size_t>(std::floor(position));
        double fraction = position - index;

        if (index == sorted_data.size() - 1) {
            return static_cast<double>(sorted_data.back());
        } else {
            return static_cast<double>(sorted_data[index]) +
                   fraction * (static_cast<double>(sorted_data[index + 1]) -
                              static_cast<double>(sorted_data[index]));
        }
    }

    /// 计算中位数 - 支持任意容器类型
    template<typename Container>
    static double median(const Container& data) {
        return quantile(data, 0.5);
    }

    /// 计算中位秩 - 支持任意容器类型
    template<typename Container>
    static double median_rank(const Container& data, T value) {
        if (data.empty()) return 0.5;

        // 创建排序副本
        std::vector<T> sorted_data(data.begin(), data.end());
        std::sort(sorted_data.begin(), sorted_data.end());

        auto it = std::lower_bound(sorted_data.begin(), sorted_data.end(), value);
        size_t rank = std::distance(sorted_data.begin(), it);

        return (rank + 0.5) / sorted_data.size();
    }

    /// 计算两个序列的相关系数 - 支持任意容器类型
    template<typename Container1, typename Container2>
    static double correlation(const Container1& x, const Container2& y) {
        static_assert(std::is_arithmetic_v<typename Container1::value_type>,
                     "Container1 value type must be arithmetic");
        static_assert(std::is_arithmetic_v<typename Container2::value_type>,
                     "Container2 value type must be arithmetic");

        if (x.size() != y.size() || x.size() < 2) return 0.0;

        double mean_x = mean(x);
        double mean_y = mean(y);
        double sum_xy = 0.0, sum_xx = 0.0, sum_yy = 0.0;

        auto x_it = x.begin();
        auto y_it = y.begin();
        for (; x_it != x.end() && y_it != y.end(); ++x_it, ++y_it) {
            double dx = static_cast<double>(*x_it) - mean_x;
            double dy = static_cast<double>(*y_it) - mean_y;
            sum_xy += dx * dy;
            sum_xx += dx * dx;
            sum_yy += dy * dy;
        }

        if (sum_xx == 0.0 || sum_yy == 0.0) return 0.0;
        return sum_xy / std::sqrt(sum_xx * sum_yy);
    }

    /// 计算协方差 - 支持任意容器类型
    template<typename Container1, typename Container2>
    static double covariance(const Container1& x, const Container2& y) {
        static_assert(std::is_arithmetic_v<typename Container1::value_type>,
                     "Container1 value type must be arithmetic");
        static_assert(std::is_arithmetic_v<typename Container2::value_type>,
                     "Container2 value type must be arithmetic");

        if (x.size() != y.size() || x.size() < 2) return 0.0;

        double mean_x = mean(x);
        double mean_y = mean(y);
        double sum_xy = 0.0;

        auto x_it = x.begin();
        auto y_it = y.begin();
        for (; x_it != x.end() && y_it != y.end(); ++x_it, ++y_it) {
            sum_xy += (static_cast<double>(*x_it) - mean_x) *
                      (static_cast<double>(*y_it) - mean_y);
        }

        return sum_xy / (x.size() - 1);
    }

    /// 计算滑动窗口统计量 - 支持任意容器类型
    template<typename Container>
    static std::vector<double> rolling_mean(const Container& data, size_t window_size) {
        std::vector<double> result;
        if (data.size() < window_size) return result;

        result.reserve(data.size() - window_size + 1);
        for (size_t i = 0; i <= data.size() - window_size; ++i) {
            std::vector<T> window;
            window.reserve(window_size);

            auto start = std::next(data.begin(), i);
            auto end = std::next(start, window_size);
            std::copy(start, end, std::back_inserter(window));

            result.push_back(mean(window));
        }

        return result;
    }
};

} // namespace math
} // namespace factorlib