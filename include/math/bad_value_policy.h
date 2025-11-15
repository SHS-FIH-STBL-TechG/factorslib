// include/math/bad_value_policy.h
#pragma once

#include <type_traits>
#include <cmath>
#include <limits>

#include "utils/log.h"

namespace factorlib {
    namespace math {

        /// 判断一个标量是否是有限数（非 NaN / 非 Inf）
        /// 对非浮点类型，统一视为“总是有限”。
        template<typename T>
        inline bool is_finite_numeric(const T& v) {
            if constexpr (std::is_floating_point_v<T>) {
                return std::isfinite(static_cast<double>(v));
            } else {
                return true;
            }
        }

        /// 策略 1：遇到 NaN / Inf 直接丢弃并记日志。
        /// handle 返回 false 表示“调用方应当丢弃这个样本，不继续处理”。
        struct SkipNaNInfPolicy {
            template<typename T>
            static bool handle(T& v, const char* where) {
                if constexpr (std::is_arithmetic_v<T>) {
                    if (!is_finite_numeric(v)) {
                        LOG_WARN("{}: skip NaN/Inf value {}", where, v);
                        return false;
                    }
                }
                return true;
            }
        };

        /// 策略 2：遇到 NaN / Inf 记日志并将其替换为 0。
        /// handle 始终返回 true，表示样本仍然保留，只是值可能被修改为 0。
        struct ZeroNaNInfPolicy {
            template<typename T>
            static bool handle(T& v, const char* where) {
                if constexpr (std::is_arithmetic_v<T>) {
                    if (!is_finite_numeric(v)) {
                        LOG_WARN("{}: NaN/Inf value {} replaced with 0", where, v);
                        v = static_cast<T>(0);
                    }
                }
                return true;
            }
        };

    } // namespace math
} // namespace factorlib
