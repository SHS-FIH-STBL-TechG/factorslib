// 说明：本文件的均值/方差/协方差/相关已改为单次遍历（Welford/Pébay），更稳更快。
// 是否增量：容器一次性计算 = 单遍在线；连续滑窗请使用 sliding_* 模块。
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
            LOG_DEBUG("Statistics::mean: skipped {} NaN values out of {}", nan_count, data.size());
        }

        if (valid_count == 0) {
            // 全部为 NaN，返回 0.0 作为退化值（与原来空容器行为保持一致）
            return 0.0;
        }

        return sum / static_cast<double>(valid_count);
    }

    /// 计算标准差 - 支持任意容器类型；NaN 视为缺失并跳过
    
/// 计算标准差 - 单次遍历（Welford），NaN 跳过
template<typename Container>
static double stddev(const Container& data) {
    long double mean = 0.0L;
    long double M2   = 0.0L;
    std::size_t n = 0;
    std::size_t nan_count = 0;

    for (const auto& x : data) {
        if (is_nan_value(x)) {
            ++nan_count;
            continue;
        }
        long double v = static_cast<long double>(x);
        ++n;
        long double dx = v - mean;
        mean += dx / static_cast<long double>(n);
        M2   += dx * (v - mean);
    }

    if (nan_count > 0) {
        LOG_DEBUG("Statistics::stddev: skipped {} NaN values out of {}", nan_count, data.size());
    }

    if (n < 2) {
        return 0.0;
    }

    long double var = M2 / static_cast<long double>(n - 1);
    return var > 0.0L ? std::sqrt(static_cast<double>(var)) : 0.0;
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
            LOG_DEBUG("Statistics::quantile: skipped {} NaN values out of {}", nan_count, data.size());
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
            LOG_DEBUG("Statistics::median_rank: input value is NaN, return 0.5 by default");
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
            LOG_DEBUG("Statistics::median_rank: skipped {} NaN values out of {}", nan_count, data.size());
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
    
/// 计算相关系数 - 单次遍历（Pébay），NaN 对跳过（任一维为 NaN 即跳过）
template<typename Container1, typename Container2>
static double correlation(const Container1& x, const Container2& y) {
    static_assert(std::is_arithmetic_v<typename Container1::value_type>,
                  "Container1 value type must be arithmetic");
    static_assert(std::is_arithmetic_v<typename Container2::value_type>,
                  "Container2 value type must be arithmetic");

    if (x.size() != y.size() || x.size() < 2) {
        return 0.0;
    }

    long double mx=0.0L, my=0.0L, C=0.0L, M2x=0.0L, M2y=0.0L;
    std::size_t n=0, nan_pair_count=0;

    auto x_it = x.begin();
    auto y_it = y.begin();
    for (; x_it != x.end() && y_it != y.end(); ++x_it, ++y_it) {
        if (is_nan_value(*x_it) || is_nan_value(*y_it)) {
            ++nan_pair_count;
            continue;
        }
        long double xi = static_cast<long double>(*x_it);
        long double yi = static_cast<long double>(*y_it);
        ++n;
        long double dx = xi - mx; mx += dx / static_cast<long double>(n);
        long double dy = yi - my; my += dy / static_cast<long double>(n);
        C  += dx * (yi - my);
        M2x+= dx * (xi - mx);
        M2y+= dy * (yi - my);
    }

    if (nan_pair_count > 0) {
        LOG_DEBUG("Statistics::correlation: skipped {} NaN pairs out of {}", nan_pair_count, x.size());
    }
    if (n < 2) {
        return 0.0;
    }

    long double vx = M2x / static_cast<long double>(n - 1);
    long double vy = M2y / static_cast<long double>(n - 1);
    if (vx <= 0.0L || vy <= 0.0L) {
        return 0.0;
    }
    long double cov = C / static_cast<long double>(n - 1);
    return static_cast<double>( cov / std::sqrt(vx * vy) );
}


    /// 计算协方差 - 支持任意容器类型；NaN 会被过滤掉（任一维为 NaN 即跳过该 pair）
    
/// 计算协方差 - 单次遍历（Pébay），NaN 对跳过（任一维为 NaN 即跳过）
template<typename Container1, typename Container2>
static double covariance(const Container1& x, const Container2& y) {
    static_assert(std::is_arithmetic_v<typename Container1::value_type>,
                  "Container1 value type must be arithmetic");
    static_assert(std::is_arithmetic_v<typename Container2::value_type>,
                  "Container2 value type must be arithmetic");

    if (x.size() != y.size() || x.size() < 2) {
        return 0.0;
    }

    long double mx = 0.0L, my = 0.0L, C = 0.0L;
    std::size_t n = 0;
    std::size_t nan_pair_count = 0;

    auto x_it = x.begin();
    auto y_it = y.begin();
    for (; x_it != x.end() && y_it != y.end(); ++x_it, ++y_it) {
        if (is_nan_value(*x_it) || is_nan_value(*y_it)) {
            ++nan_pair_count;
            continue;
        }
        long double xi = static_cast<long double>(*x_it);
        long double yi = static_cast<long double>(*y_it);
        ++n;
        long double dx = xi - mx; mx += dx / static_cast<long double>(n);
        long double dy = yi - my; my += dy / static_cast<long double>(n);
        C += dx * (yi - my);
    }

    if (nan_pair_count > 0) {
        LOG_DEBUG("Statistics::covariance: skipped {} NaN pairs out of {}", nan_pair_count, x.size());
    }
    if (n < 2) {
        return 0.0;
    }
    return static_cast<double>( C / static_cast<long double>(n - 1) );
}


    
/// 计算滑动窗口均值 - O(n) 前缀和实现；每个窗口内部 NaN 被过滤掉
template<typename Container>
static std::vector<double> rolling_mean(const Container& data, std::size_t window_size) {
    std::vector<double> result;
    const std::size_t N = data.size();
    if (N < window_size || window_size == 0) {
        return result;
    }

    // 将输入拷到数组，便于前缀和；同时记录有效性
    std::vector<double> vals; vals.reserve(N);
    std::vector<int>    valid; valid.reserve(N);
    for (const auto& x : data) {
        if (is_nan_value(x)) {
            vals.push_back(0.0);
            valid.push_back(0);
        } else {
            vals.push_back(static_cast<double>(x));
            valid.push_back(1);
        }
    }

    // 前缀和：ps[i]=sum_{0..i-1} vals[j]*valid[j]；pc[i]=count of valid
    std::vector<long double> ps(N+1, 0.0L);
    std::vector<int>         pc(N+1, 0);
    for (std::size_t i=0;i<N;++i) {
        ps[i+1] = ps[i] + static_cast<long double>(vals[i]) * static_cast<long double>(valid[i]);
        pc[i+1] = pc[i] + valid[i];
    }

    result.reserve(N - window_size + 1);
    for (std::size_t i=0; i+window_size<=N; ++i) {
        long double sum  = ps[i+window_size] - ps[i];
        int         cnt  = pc[i+window_size] - pc[i];
        if (cnt <= 0) {
            result.push_back(0.0);
        } else {
            result.push_back(static_cast<double>(sum / static_cast<long double>(cnt)));
        }
    }
    return result;
}

};

} // namespace math
} // namespace factorlib
