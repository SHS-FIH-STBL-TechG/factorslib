
#pragma once
#include <cstdarg>
#include <cstdio>

/**
 * @file log.h
 * @brief 日志对外头文件，提供三个宏：TSET_INFO/TSET_WARN/TSET_ERROR。
 * 使用方式：在代码中直接调用 TSET_INFO("a=%d", a);
 * 输出格式为：[LEVEL] [函数签名] 消息文本
 */

namespace factorlib {

/// 日志级别
enum class LogLevel { INFO, WARN, ERROR };

/**
 * @brief 记录一条日志（内部实现函数，建议使用宏）
 * @param lvl   日志级别
 * @param funcSig 当前函数签名（由宏传入）
 * @param fmt   printf 风格的格式串
 * @param ...   可变参数
 */
void log_message(LogLevel lvl, const char* funcSig, const char* fmt, ...);

} // namespace factorlib

// 下面的宏自动传入函数签名，便于定位问题
#if defined(__GNUC__) || defined(__clang__)
  #define TSET_INFO(...)  ::factorlib::log_message(::factorlib::LogLevel::INFO, __PRETTY_FUNCTION__, __VA_ARGS__)
  #define TSET_WARN(...)  ::factorlib::log_message(::factorlib::LogLevel::WARN,  __PRETTY_FUNCTION__, __VA_ARGS__)
  #define TSET_ERROR(...) ::factorlib::log_message(::factorlib::LogLevel::ERROR, __PRETTY_FUNCTION__, __VA_ARGS__)
#else
  #define TSET_INFO(...)  ::factorlib::log_message(::factorlib::LogLevel::INFO, __FUNCTION__, __VA_ARGS__)
  #define TSET_WARN(...)  ::factorlib::log_message(::factorlib::LogLevel::WARN,  __FUNCTION__, __VA_ARGS__)
  #define TSET_ERROR(...) ::factorlib::log_message(::factorlib::LogLevel::ERROR, __FUNCTION__, __VA_ARGS__)
#endif
