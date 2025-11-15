// include/utils/math/distributions.h
#pragma once
#include <cmath>
#include <vector>
#include <deque>
#include <algorithm>
#include <type_traits>
#include <iterator>

#include "utils/log.h"

#if __has_include(<boost/math/distributions/fisher_f.hpp>)
  #include <boost/math/distributions/fisher_f.hpp>
  #define FACTORLIB_HAS_BOOST_MATH 1
#else
  #define FACTORLIB_HAS_BOOST_MATH 0
#endif

namespace factorlib {
namespace math {

/**
 * @brief 概率分布计算模板类
 */
class Distributions {
public:
    /**
     * @brief 正态分布逆CDF（使用Wichura算法近似）
     *        - 若 p 非有限或不在 (0,1) 内，返回 0.0 并给出 warning
     */
    static double normal_quantile(double p) {
        if (!std::isfinite(p)) {
            LOG_WARN("Distributions::normal_quantile: p is NaN/inf ({}) , return 0.0", p);
            return 0.0;
        }

        if (p <= 0.0 || p >= 1.0) {
            LOG_WARN("Distributions::normal_quantile: p out of range ({}) , return 0.0", p);
            return 0.0;
        }

        static const double a1 = -3.969683028665376e+01;
        static const double a2 =  2.209460984245205e+02;
        static const double a3 = -2.759285104469687e+02;
        static const double a4 =  1.383577518672690e+02;
        static const double a5 = -3.066479806614716e+01;
        static const double a6 =  2.506628277459239e+00;

        static const double b1 = -5.447609879822406e+01;
        static const double b2 =  1.615858368580409e+02;
        static const double b3 = -1.556989798598866e+02;
        static const double b4 =  6.680131188771972e+01;
        static const double b5 = -1.328068155288572e+01;

        static const double c1 = -7.784894002430293e-03;
        static const double c2 = -3.223964580411365e-01;
        static const double c3 = -2.400758277161838e+00;
        static const double c4 = -2.549732539343734e+00;
        static const double c5 =  4.374664141464968e+00;
        static const double c6 =  2.938163982698783e+00;

        static const double d1 =  7.784695709041462e-03;
        static const double d2 =  3.224671290700398e-01;
        static const double d3 =  2.445134137142996e+00;
        static const double d4 =  3.754408661907416e+00;

        double q, r;

        if (p < 0.02425) {
            q = std::sqrt(-2.0 * std::log(p));
            return (((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
                   ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
        } else if (p > 0.97575) {
            q = std::sqrt(-2.0 * std::log(1.0 - p));
            return -(((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
                    ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
        } else {
            q = p - 0.5;
            r = q * q;
            return (((((a1 * r + a2) * r + a3) * r + a4) * r + a5) * r + a6) * q /
                   (((((b1 * r + b2) * r + b3) * r + b4) * r + b5) * r + 1.0);
        }
    }

    /**
     * @brief 标准正态分布 CDF：Φ(z) = P(Z <= z), Z ~ N(0,1)
     * @tparam T 浮点类型，如 float/double/long double
     */
    template<typename T>
    static T normal_cdf(T z) {
        static_assert(std::is_floating_point_v<T>,
                      "Distributions::normal_cdf requires floating point type");

        double zd = static_cast<double>(z);
        if (!std::isfinite(zd)) {
            LOG_WARN("Distributions::normal_cdf: z is NaN/inf ({}) , return 0.5", zd);
            return static_cast<T>(0.5);
        }

        double F = 0.5 * std::erfc(-zd / std::sqrt(2.0));
        return static_cast<T>(F);
    }

    /**
     * @brief 经验逆CDF计算 - 支持任意容器类型
     *
     * NaN 视为“缺失值”，在样本中会被跳过：
     *  - 若存在 NaN，则打印一条 warning；
     *  - 若全部为 NaN（清洗后无有效样本），返回 0.0。
     *
     * 概率 probability：
     *  - 若非有限或不在 [0,1] 内，返回 0.0 并打印 warning。
     */
    template<typename Container>
    static double empirical_inverse_cdf(const Container& data, double probability) {
        using ValueType = typename Container::value_type;
        static_assert(std::is_arithmetic_v<ValueType>, "Container value type must be arithmetic");

        if (data.empty()) {
            return 0.0;
        }

        if (!std::isfinite(probability) || probability < 0.0 || probability > 1.0) {
            LOG_WARN("Distributions::empirical_inverse_cdf: invalid probability {} , return 0.0", probability);
            return 0.0;
        }

        // 过滤 NaN，构建清洗后的样本
        std::vector<double> cleaned;
        cleaned.reserve(data.size());
        std::size_t nan_count = 0;

        for (const auto& x : data) {
            using Decayed = std::decay_t<ValueType>;
            if constexpr (std::is_floating_point_v<Decayed>) {
                if (std::isnan(static_cast<double>(x))) {
                    ++nan_count;
                    continue;
                }
            }
            cleaned.push_back(static_cast<double>(x));
        }

        if (nan_count > 0) {
            LOG_WARN("Distributions::empirical_inverse_cdf: skipped {} NaN values out of {}",
                     nan_count, data.size());
        }

        if (cleaned.empty()) {
            // 全部为 NaN，退化返回 0.0（与其它统计函数的退化行为保持一致）
            return 0.0;
        }

        // 创建排序副本
        std::sort(cleaned.begin(), cleaned.end());

        // 计算分位数位置
        const double position = probability * (static_cast<double>(cleaned.size()) - 1.0);
        const std::size_t index = static_cast<std::size_t>(std::floor(position));
        const double fraction = position - static_cast<double>(index);

        // 线性插值
        if (index >= cleaned.size() - 1) {
            return cleaned.back();
        } else {
            const double lower = cleaned[index];
            const double upper = cleaned[index + 1];
            return lower + fraction * (upper - lower);
        }
    }
};

/**
 * @brief F 分布右尾概率（Survival Function）：Pr(F_{d1,d2} >= Fobs)
 *        - 模板化以支持 float/double/long double
 *        - 优先使用 Boost.Math（header-only），无 Boost 时保守返回 1
 *
 *  NaN 处理：
 *   - 若 Fobs 非有限（NaN/inf）或 d1/d2 非法，返回 1 并打印 warning。
 *
 * @tparam TFloat 浮点类型（float/double/long double）
 * @param Fobs    观察到的 F 值（>=0）
 * @param d1      自由度1（通常是受限与非受限的参数差 q）
 * @param d2      自由度2（通常是非受限模型的残差自由度 N - p - q - 1）
 * @return 右尾概率 p ∈ [0,1]；无 Boost 时返回 1（表示“不显著”）
 */
template<typename TFloat>
inline TFloat fisher_f_sf(TFloat Fobs, int d1, int d2) {
    static_assert(std::is_floating_point_v<TFloat>, "TFloat must be floating point");

#if FACTORLIB_HAS_BOOST_MATH
    if (!std::isfinite(static_cast<double>(Fobs))) {
        LOG_WARN("Distributions::fisher_f_sf: Fobs is NaN/inf ({}) , return 1", Fobs);
        return static_cast<TFloat>(1);
    }
    if (!(Fobs >= static_cast<TFloat>(0)) || d1 <= 0 || d2 <= 0) {
        LOG_WARN("Distributions::fisher_f_sf: invalid args Fobs = {}, d1 = {}, d2 = {} , return 1",
                 Fobs, d1, d2);
        return static_cast<TFloat>(1);
    }

    try {
        boost::math::fisher_f dist(d1, d2);
        // 右尾概率 = cdf_complement(Fobs)
        TFloat p = boost::math::cdf(boost::math::complement(dist, Fobs));
        if (p < static_cast<TFloat>(0)) p = static_cast<TFloat>(0);
        if (p > static_cast<TFloat>(1)) p = static_cast<TFloat>(1);
        return p;
    } catch (...) {
        // 任何异常（数值/参数）时返回保守值
        LOG_WARN("Distributions::fisher_f_sf: exception caught in boost::math, return 1");
        return static_cast<TFloat>(1);
    }
#else
    // 无 Boost：保守退化，返回 1（不显著）
    (void)Fobs; (void)d1; (void)d2;
    LOG_WARN("Distributions::fisher_f_sf: Boost.Math not available, return 1");
    return static_cast<TFloat>(1);
#endif
}

/**
 * @brief 便捷别名：与 fisher_f_sf 相同（可读性更强）
 */
template<typename TFloat>
inline TFloat fisher_f_pvalue_right_tail(TFloat Fobs, int d1, int d2) {
    return fisher_f_sf<TFloat>(Fobs, d1, d2);
}

} // namespace math
} // namespace factorlib
