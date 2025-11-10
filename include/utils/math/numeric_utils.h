// include/utils/math/numeric_utils.h
#pragma once
#include <cmath>
#include <type_traits>
#include <limits>

namespace factorlib {
namespace math {

/**
 * @brief 数值计算工具模板类
 */
template<typename T>
class NumericUtils {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

public:
    /// 安全的对数计算（处理非正数）
    static double safe_log(T x) {
        return x > 0 ? std::log(static_cast<double>(x)) :
                       std::log(std::numeric_limits<double>::min());
    }

    /// 安全的除法（避免除零）
    static double safe_divide(T numerator, T denominator) {
        if (denominator == 0) return 0.0;
        return static_cast<double>(numerator) / static_cast<double>(denominator);
    }

    /// 限制数值在指定范围内
    static T clamp(T value, T min_val, T max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }

    /// 检查数值是否接近零（考虑浮点精度）
    static bool is_near_zero(T value, T epsilon = std::numeric_limits<T>::epsilon()) {
        return std::abs(value) < epsilon;
    }

    /// 线性插值
    static double lerp(T a, T b, double t) {
        return (1.0 - t) * static_cast<double>(a) + t * static_cast<double>(b);
    }

    /// 计算对数收益率
    template<typename U>
    static double log_return(U current_price, U previous_price) {
        static_assert(std::is_arithmetic_v<U>, "U must be an arithmetic type");

        if (previous_price == 0) return 0.0;
        return std::log(static_cast<double>(current_price) /
                        static_cast<double>(previous_price));
    }

    /// 计算简单收益率
    template<typename U>
    static double simple_return(U current_price, U previous_price) {
        static_assert(std::is_arithmetic_v<U>, "U must be an arithmetic type");

        if (previous_price == 0) return 0.0;
        return (static_cast<double>(current_price) -
                static_cast<double>(previous_price)) /
                static_cast<double>(previous_price);
    }
};

} // namespace math
} // namespace factorlib