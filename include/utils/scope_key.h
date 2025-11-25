// include/utils/scope_key.h
#pragma once
#include <string>
#include <cstdint>

namespace factorlib {

/**
 * @brief 描述一组“code + 时间频率 + 窗口”组合键。
 *        - code       : 原始标的代码
 *        - frequency  : 时间频率（取样间隔，1 表示逐条）
 *        - window     : 窗口大小（若无窗口概念可置 0）
 *
 * 该组合在 DataBus / 状态映射等场景中作为唯一键，确保维度完全匹配。
 * 增加显式结构可以避免字符串拼接散落在各处，方便统一生成/解析。
 */
struct ScopeKey {
    std::string code;
    int64_t frequency{1}; ///< 频率对应的毫秒周期
    int window{0};

    ScopeKey() = default;
    ScopeKey(std::string c, int f, int w)
        : code(std::move(c)), frequency(f), window(w) {}

    /**
     * @brief 返回统一格式的组合键字符串（code|fX|wY）。
     *        调用方可直接将其作为 DataBus 的 code 维度使用。
     */
    std::string as_bus_code() const;
};

/// @brief 统一的组合键格式：code|f{frequency}|w{window}
/// @brief 工具函数：使用“code|f{freq}|w{win}”格式拼接组合键
inline std::string compose_scope_code(const std::string& code, int64_t frequency, int window) {
    return code + "|f" + std::to_string(frequency) + "|w" + std::to_string(window);
}

inline std::string ScopeKey::as_bus_code() const {
    return compose_scope_code(code, frequency, window);
}

/**
 * @brief 解析 DataBus code 字符串还原 ScopeKey。
 *        - 如果 code 中不存在 “|f / |w”，则保持原样并回退到默认频率和窗口。
 */
inline ScopeKey parse_scope_code(const std::string& scoped) {
    ScopeKey scope;
    scope.code = scoped;
    auto pos_f = scoped.rfind("|f");
    auto pos_w = scoped.rfind("|w");
    if (pos_f != std::string::npos && pos_w != std::string::npos && pos_f < pos_w) {
        scope.code = scoped.substr(0, pos_f);
        try {
            scope.frequency = std::stoll(scoped.substr(pos_f + 2, pos_w - pos_f - 2));
            scope.window    = std::stoi(scoped.substr(pos_w + 2));
        } catch (...) {
            scope.frequency = 1;
            scope.window    = 0;
        }
    }
    return scope;
}

} // namespace factorlib
