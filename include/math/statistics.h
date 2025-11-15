// include/utils/math/statistics.h
#pragma once
#include <vector>
#include <deque>
#include <algorithm>
#include <numeric>
#include <type_traits>
#include <cmath>
#include <iterator>

#include "utils/log.h"

namespace factorlib {
namespace math {

/**
 * @brief 统计计算模板类 - 支持任意数值类型和容器类型
 *
 * NaN 视为“缺失值”：在统计量计算中会被跳过，并输出一次 WARN 日志。
 */
template<typename T>
class Statistics {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

    /// 判断单个值是否为 NaN（仅对浮点类型有意义）
    template<typename U>
    static bool is_nan_value(const U& v) {
        using Decayed = std::decay_t<U>;
        if constexpr (std::is_floating_point_v<Decayed>) {
            return std::isnan(static_cast<double>(v));
        } else {
            return false;
        }
    }

public:
    /// 计算均值 - 支持任意容器类型；NaN 视为缺失并跳过
    template<typename Container>
    static double mean(const Container& data) {
        if (data.empty()) {
            return 0.0;
        }

        double sum = 0.0;
        std::size_t valid_count = 0;
        std::size_t nan_count = 0;

        for (const auto& x : data) {
            if (is_nan_value(x)) {
                ++nan_count;
                continue;
            }
            sum += static_cast<double>(x);
            ++valid_count;
        }

        if (nan_count > 0) {
            LOG_WARN("Statistics::mean: skipped {} NaN values out of {}", nan_count, data.size());
        }

        if (valid_count == 0) {
            // 全部为 NaN，返回 0.0 作为退化值（与原来空容器行为保持一致）
            return 0.0;
        }

        return sum / static_cast<double>(valid_count);
    }

    /// 计算标准差 - 支持任意容器类型；NaN 视为缺失并跳过
    template<typename Container>
    static double stddev(const Container& data) {
        if (data.size() < 2) {
            return 0.0;
        }

        // 先按“跳过 NaN”的规则计算均值
        double m = 0.0;
        {
            double sum = 0.0;
            std::size_t valid_count = 0;
            for (const auto& x : data) {
                if (is_nan_value(x)) {
                    continue;
                }
                sum += static_cast<double>(x);
                ++valid_count;
            }
            if (valid_count < 2) {
                return 0.0;
            }
            m = sum / static_cast<double>(valid_count);
        }

        double sum_sq = 0.0;
        std::size_t valid_count = 0;
        std::size_t nan_count = 0;

        for (const auto& x : data) {
            if (is_nan_value(x)) {
                ++nan_count;
                continue;
            }
            double dx = static_cast<double>(x) - m;
            sum_sq += dx * dx;
            ++valid_count;
        }

        if (nan_count > 0) {
            LOG_WARN("Statistics::stddev: skipped {} NaN values out of {}", nan_count, data.size());
        }

        if (valid_count < 2) {
            return 0.0;
        }

        // 样本标准差：除以 (n - 1)
        return std::sqrt(sum_sq / static_cast<double>(valid_count - 1));
    }

    /// 计算分位数 - 支持任意容器类型；NaN 会被过滤掉
    template<typename Container>
    static double quantile(const Container& data, double percentile) {
        if (data.empty()) {
            return 0.0;
        }

        std::vector<double> cleaned;
        cleaned.reserve(data.size());
        std::size_t nan_count = 0;

        for (const auto& x : data) {
            if (is_nan_value(x)) {
                ++nan_count;
            } else {
                cleaned.push_back(static_cast<double>(x));
            }
        }

        if (nan_count > 0) {
            LOG_WARN("Statistics::quantile: skipped {} NaN values out of {}", nan_count, data.size());
        }

        if (cleaned.empty()) {
            return 0.0;
        }

        std::sort(cleaned.begin(), cleaned.end());

        const double position = percentile * (static_cast<double>(cleaned.size()) - 1.0);
        const std::size_t index = static_cast<std::size_t>(std::floor(position));
        const double fraction = position - static_cast<double>(index);

        if (index >= cleaned.size() - 1) {
            return cleaned.back();
        }

        const double lower = cleaned[index];
        const double upper = cleaned[index + 1];
        return lower + fraction * (upper - lower);
    }

    /// 计算中位数 - 支持任意容器类型；NaN 会被过滤掉
    template<typename Container>
    static double median(const Container& data) {
        return quantile(data, 0.5);
    }

    /// 计算中位秩 - 支持任意容器类型；NaN 会被过滤掉
    template<typename Container>
    static double median_rank(const Container& data, T value) {
        if (data.empty()) {
            return 0.5;
        }

        if (is_nan_value(value)) {
            LOG_WARN("Statistics::median_rank: input value is NaN, return 0.5 by default");
            return 0.5;
        }

        std::vector<double> cleaned;
        cleaned.reserve(data.size());
        std::size_t nan_count = 0;

        for (const auto& x : data) {
            if (is_nan_value(x)) {
                ++nan_count;
            } else {
                cleaned.push_back(static_cast<double>(x));
            }
        }

        if (nan_count > 0) {
            LOG_WARN("Statistics::median_rank: skipped {} NaN values out of {}", nan_count, data.size());
        }

        if (cleaned.empty()) {
            return 0.5;
        }

        std::sort(cleaned.begin(), cleaned.end());

        auto it = std::lower_bound(cleaned.begin(), cleaned.end(), static_cast<double>(value));
        const std::size_t rank = static_cast<std::size_t>(std::distance(cleaned.begin(), it));

        return (static_cast<double>(rank) + 0.5) / static_cast<double>(cleaned.size());
    }

    /// 计算两个序列的相关系数 - 支持任意容器类型；NaN 会被过滤掉（任一维为 NaN 即跳过该 pair）
    template<typename Container1, typename Container2>
    static double correlation(const Container1& x, const Container2& y) {
        static_assert(std::is_arithmetic_v<typename Container1::value_type>,
                      "Container1 value type must be arithmetic");
        static_assert(std::is_arithmetic_v<typename Container2::value_type>,
                      "Container2 value type must be arithmetic");

        if (x.size() != y.size() || x.size() < 2) {
            return 0.0;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        std::size_t valid_count = 0;
        std::size_t nan_pair_count = 0;

        auto x_it = x.begin();
        auto y_it = y.begin();
        for (; x_it != x.end() && y_it != y.end(); ++x_it, ++y_it) {
            if (is_nan_value(*x_it) || is_nan_value(*y_it)) {
                ++nan_pair_count;
                continue;
            }
            sum_x += static_cast<double>(*x_it);
            sum_y += static_cast<double>(*y_it);
            ++valid_count;
        }

        if (nan_pair_count > 0) {
            LOG_WARN("Statistics::correlation: skipped {} NaN pairs out of {}", nan_pair_count, x.size());
        }

        if (valid_count < 2) {
            return 0.0;
        }

        const double mean_x = sum_x / static_cast<double>(valid_count);
        const double mean_y = sum_y / static_cast<double>(valid_count);

        double sum_xy = 0.0;
        double sum_xx = 0.0;
        double sum_yy = 0.0;

        x_it = x.begin();
        y_it = y.begin();
        for (; x_it != x.end() && y_it != y.end(); ++x_it, ++y_it) {
            if (is_nan_value(*x_it) || is_nan_value(*y_it)) {
                continue;
            }
            const double dx = static_cast<double>(*x_it) - mean_x;
            const double dy = static_cast<double>(*y_it) - mean_y;
            sum_xy += dx * dy;
            sum_xx += dx * dx;
            sum_yy += dy * dy;
        }

        if (sum_xx == 0.0 || sum_yy == 0.0) {
            return 0.0;
        }
        return sum_xy / std::sqrt(sum_xx * sum_yy);
    }

    /// 计算协方差 - 支持任意容器类型；NaN 会被过滤掉（任一维为 NaN 即跳过该 pair）
    template<typename Container1, typename Container2>
    static double covariance(const Container1& x, const Container2& y) {
        static_assert(std::is_arithmetic_v<typename Container1::value_type>,
                      "Container1 value type must be arithmetic");
        static_assert(std::is_arithmetic_v<typename Container2::value_type>,
                      "Container2 value type must be arithmetic");

        if (x.size() != y.size() || x.size() < 2) {
            return 0.0;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        std::size_t valid_count = 0;
        std::size_t nan_pair_count = 0;

        auto x_it = x.begin();
        auto y_it = y.begin();
        for (; x_it != x.end() && y_it != y.end(); ++x_it, ++y_it) {
            if (is_nan_value(*x_it) || is_nan_value(*y_it)) {
                ++nan_pair_count;
                continue;
            }
            sum_x += static_cast<double>(*x_it);
            sum_y += static_cast<double>(*y_it);
            ++valid_count;
        }

        if (nan_pair_count > 0) {
            LOG_WARN("Statistics::covariance: skipped {} NaN pairs out of {}", nan_pair_count, x.size());
        }

        if (valid_count < 2) {
            return 0.0;
        }

        const double mean_x = sum_x / static_cast<double>(valid_count);
        const double mean_y = sum_y / static_cast<double>(valid_count);

        double sum_xy = 0.0;

        x_it = x.begin();
        y_it = y.begin();
        for (; x_it != x.end() && y_it != y.end(); ++x_it, ++y_it) {
            if (is_nan_value(*x_it) || is_nan_value(*y_it)) {
                continue;
            }
            sum_xy += (static_cast<double>(*x_it) - mean_x) *
                      (static_cast<double>(*y_it) - mean_y);
        }

        return sum_xy / static_cast<double>(valid_count - 1);
    }

    /// 计算滑动窗口均值 - 支持任意容器类型；每个窗口内部 NaN 被过滤掉
    template<typename Container>
    static std::vector<double> rolling_mean(const Container& data, std::size_t window_size) {
        std::vector<double> result;
        if (data.size() < window_size || window_size == 0) {
            return result;
        }

        result.reserve(data.size() - window_size + 1);
        for (std::size_t i = 0; i <= data.size() - window_size; ++i) {
            std::vector<double> window;
            window.reserve(window_size);

            auto start = std::next(data.begin(), static_cast<std::ptrdiff_t>(i));
            auto end   = std::next(start, static_cast<std::ptrdiff_t>(window_size));
            for (auto it = start; it != end; ++it) {
                if (!is_nan_value(*it)) {
                    window.push_back(static_cast<double>(*it));
                }
            }

            if (window.empty()) {
                // 整个窗口都是 NaN，返回 0.0（与整体风格统一）
                result.push_back(0.0);
            } else {
                result.push_back(mean(window));
            }
        }

        return result;
    }
};

} // namespace math
} // namespace factorlib
