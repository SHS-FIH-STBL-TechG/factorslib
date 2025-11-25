// include/utils/math/numeric_utils.h
#pragma once
#include <cmath>
#include <type_traits>
#include <limits>
#include <algorithm>
#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

/**
 * @brief 数值计算工具模板类
 * @tparam T 数值类型，必须是算术类型
 * @details 提供高频因子计算中常用的数值计算方法，集成坏值处理策略
 */
template<typename T>
class NumericUtils {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

public:
    /**
     * @brief 安全的对数计算（处理非正数和坏值）
     * @param x 输入数值
     * @param where 错误位置标识
     * @return 计算结果，对于非正数返回最小双精度对数值
     * @details 处理非正数输入，避免数学错误，同时集成坏值检测
     */
    template<typename BadValuePolicy = SkipNaNInfPolicy>
    static double safe_log(T x, const char* where = "safe_log") {
        T temp = x;
        if (!BadValuePolicy::handle(temp, where)) {
            return std::log(std::numeric_limits<double>::min());
        }
        return temp > 0 ? std::log(static_cast<double>(temp)) :
                          std::log(std::numeric_limits<double>::min());
    }

    /**
     * @brief 安全的除法运算（避免除零和坏值）
     * @param numerator 分子
     * @param denominator 分母
     * @param where 错误位置标识
     * @return 除法结果，分母为零时返回0.0
     * @details 防止除零错误，同时处理输入的坏值情况
     */
    template<typename BadValuePolicy = SkipNaNInfPolicy>
    static double safe_divide(T numerator, T denominator, const char* where = "safe_divide") {
        T num_temp = numerator;
        T den_temp = denominator;

        bool num_valid = BadValuePolicy::handle(num_temp, (std::string(where) + "_numerator").c_str());
        bool den_valid = BadValuePolicy::handle(den_temp, (std::string(where) + "_denominator").c_str());

        if (!num_valid || !den_valid) {
            return 0.0;
        }

        if (den_temp == 0) return 0.0;
        return static_cast<double>(num_temp) / static_cast<double>(den_temp);
    }

    /**
     * @brief 限制数值在指定范围内
     * @param value 待限制的数值
     * @param min_val 最小值
     * @param max_val 最大值
     * @param where 错误位置标识
     * @return 限制后的数值
     * @details 确保数值在[min_val, max_val]范围内，处理输入坏值
     */
    template<typename BadValuePolicy = SkipNaNInfPolicy>
    static T clamp(T value, T min_val, T max_val, const char* where = "clamp") {
        T val_temp = value;
        T min_temp = min_val;
        T max_temp = max_val;

        if (!BadValuePolicy::handle(val_temp, (std::string(where) + "_value").c_str()) ||
            !BadValuePolicy::handle(min_temp, (std::string(where) + "_min").c_str()) ||
            !BadValuePolicy::handle(max_temp, (std::string(where) + "_max").c_str())) {
            return static_cast<T>(0);
        }

        if (val_temp < min_temp) return min_temp;
        if (val_temp > max_temp) return max_temp;
        return val_temp;
    }

    /**
     * @brief 检查数值是否接近零（考虑浮点精度）
     * @param value 待检查数值
     * @param epsilon 精度阈值，默认为类型T的机器精度
     * @param where 错误位置标识
     * @return 如果数值绝对值小于epsilon返回true，否则返回false
     * @details 用于判断数值是否在误差范围内接近零，处理坏值输入
     */
    template<typename BadValuePolicy = SkipNaNInfPolicy>
    static bool is_near_zero(T value, T epsilon = std::numeric_limits<T>::epsilon(),
                           const char* where = "is_near_zero") {
        T val_temp = value;
        if (!BadValuePolicy::handle(val_temp, where)) {
            return false;
        }
        return std::abs(val_temp) < epsilon;
    }

    /**
     * @brief 线性插值计算
     * @param a 起始值
     * @param b 结束值
     * @param t 插值参数，范围[0,1]
     * @param where 错误位置标识
     * @return 插值结果
     * @details 在a和b之间进行线性插值，t=0返回a，t=1返回b
     */
    template<typename BadValuePolicy = SkipNaNInfPolicy>
    static double lerp(T a, T b, double t, const char* where = "lerp") {
        T a_temp = a;
        T b_temp = b;

        bool a_valid = BadValuePolicy::handle(a_temp, (std::string(where) + "_a").c_str());
        bool b_valid = BadValuePolicy::handle(b_temp, (std::string(where) + "_b").c_str());

        if (!a_valid || !b_valid) {
            return 0.0;
        }

        // 限制t在[0,1]范围内
        t = std::clamp(t, 0.0, 1.0);
        return (1.0 - t) * static_cast<double>(a_temp) + t * static_cast<double>(b_temp);
    }

    /**
     * @brief 计算对数收益率
     * @tparam U 价格数据类型，必须是算术类型
     * @param current_price 当前价格
     * @param previous_price 前一期价格
     * @return 对数收益率
     * @details 计算ln(current/previous)，处理零价格情况
     */
    template<typename U>
    static double log_return(U current_price, U previous_price) {
        static_assert(std::is_arithmetic_v<U>, "U must be an arithmetic type");

        if (previous_price == 0) return 0.0;
        double ratio = static_cast<double>(current_price) / static_cast<double>(previous_price);
        if (ratio <= 0) return 0.0; // 避免对非正数取对数
        return std::log(ratio);
    }

    /**
     * @brief 计算带坏值处理的对数收益率
     * @param current_price 当前价格
     * @param previous_price 前一期价格
     * @param where 错误位置标识
     * @return 对数收益率
     * @details 计算ln(current/previous)，处理零价格和坏值情况
     */
    template<typename BadValuePolicy = SkipNaNInfPolicy>
    static double log_return_with_policy(T current_price, T previous_price, const char* where = "log_return") {
        T curr_temp = current_price;
        T prev_temp = previous_price;

        bool curr_valid = BadValuePolicy::handle(curr_temp, (std::string(where) + "_current").c_str());
        bool prev_valid = BadValuePolicy::handle(prev_temp, (std::string(where) + "_previous").c_str());

        if (!curr_valid || !prev_valid) {
            return 0.0;
        }

        if (prev_temp == 0) return 0.0;
        double ratio = static_cast<double>(curr_temp) / static_cast<double>(prev_temp);
        if (ratio <= 0) return 0.0; // 避免对非正数取对数
        return std::log(ratio);
    }

    /**
     * @brief 计算简单收益率
     * @tparam U 价格数据类型，必须是算术类型
     * @param current_price 当前价格
     * @param previous_price 前一期价格
     * @return 简单收益率
     * @details 计算(current-previous)/previous，处理零价格情况
     */
    template<typename U>
    static double simple_return(U current_price, U previous_price) {
        static_assert(std::is_arithmetic_v<U>, "U must be an arithmetic type");

        if (previous_price == 0) return 0.0;
        return (static_cast<double>(current_price) - static_cast<double>(previous_price)) /
                static_cast<double>(previous_price);
    }

    /**
     * @brief 计算带坏值处理的简单收益率
     * @param current_price 当前价格
     * @param previous_price 前一期价格
     * @param where 错误位置标识
     * @return 简单收益率
     * @details 计算(current-previous)/previous，处理零价格和坏值情况
     */
    template<typename BadValuePolicy = SkipNaNInfPolicy>
    static double simple_return_with_policy(T current_price, T previous_price, const char* where = "simple_return") {
        T curr_temp = current_price;
        T prev_temp = previous_price;

        bool curr_valid = BadValuePolicy::handle(curr_temp, (std::string(where) + "_current").c_str());
        bool prev_valid = BadValuePolicy::handle(prev_temp, (std::string(where) + "_previous").c_str());

        if (!curr_valid || !prev_valid) {
            return 0.0;
        }

        if (prev_temp == 0) return 0.0;
        return (static_cast<double>(curr_temp) - static_cast<double>(prev_temp)) /
                static_cast<double>(prev_temp);
    }
};

} // namespace math
} // namespace factorlib