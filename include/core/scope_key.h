// include/core/scope_key.h
#pragma once
#include <string>
#include <cstdint>

namespace factorlib {

/**
 * @brief 描述一组“code + 窗口”组合键。
 *
 * 所有因子统一使用此键作为 DataBus/状态映射的唯一标识：
 *   scoped_code = code|w{window}
 */
struct ScopeKey {
    std::string code;
    int window{0};

    ScopeKey() = default;
    ScopeKey(std::string c, int w)
        : code(std::move(c)), window(w) {}

    /**
     * @brief 返回统一格式的组合键字符串（code|wY）。
     *        调用方可直接将其作为 DataBus 的 code 维度使用。
     */
    std::string as_bus_code() const;
};

/// @brief 工具函数：使用“code|w{win}”格式拼接组合键
inline std::string compose_scope_code(const std::string& code, int window) {
    return code + "|w" + std::to_string(window);
}

inline std::string ScopeKey::as_bus_code() const {
    return compose_scope_code(code, window);
}

/**
 * @brief 解析 DataBus code 字符串还原 ScopeKey。
 *        - 对旧格式（code|fX|wY）和新格式（code|wY）都能兼容解析。
 */
inline ScopeKey parse_scope_code(const std::string& scoped) {
    ScopeKey scope;
    scope.code = scoped;
    auto pos_w = scoped.rfind("|w");
    if (pos_w != std::string::npos) {
        std::string prefix = scoped.substr(0, pos_w);
        scope.code = prefix;
        auto pos_f = prefix.rfind("|f");
        if (pos_f != std::string::npos) {
            scope.code = prefix.substr(0, pos_f);
        }
        try {
            scope.window = std::stoi(scoped.substr(pos_w + 2));
        } catch (...) {
            scope.window = 0;
        }
    }
    return scope;
}

} // namespace factorlib
