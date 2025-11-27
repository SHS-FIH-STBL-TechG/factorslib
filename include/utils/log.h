// include/utils/log.h
#pragma once
/**
 * Simple logging facade for factorlib.
 * If USE_SPDLOG is defined and headers are available, map to spdlog.
 * Otherwise fall back to fprintf(stderr, ...).
 */

// ---- Windows 头文件宏控制（需在包含任何可能引入 windows.h 的头之前定义）----
#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#endif

#ifdef USE_SPDLOG
  #include <spdlog/spdlog.h>
  #include <spdlog/sinks/stdout_color_sinks.h>
#endif
#include <cstdio>
#include <cstdarg>

namespace factorlib {
namespace log {
  // 定义日志级别
// 注意：windows.h 会定义 ERROR/WARN 等宏，需先取消定义以避免与枚举成员冲突
#if defined(_WIN32) && defined(ERROR)
#  undef ERROR
#endif
#if defined(_WIN32) && defined(WARN)
#  undef WARN
#endif
  enum class LogLevel {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    OFF = 5
};

  // 获取当前日志级别
  inline LogLevel get_log_level() {
#if defined(FACTORLIB_NO_DEBUG_TRACE)
    // 编译期裁掉 TRACE/DEBUG 时，运行时级别固定为 INFO
    return LogLevel::INFO;
#elif defined(NDEBUG)
    // Release模式默认级别
    return LogLevel::INFO;
#else
    // Debug模式默认级别
    return LogLevel::DEBUG;
#endif
  }

  // initialize logging once per process (no-op for fallback)
  inline void init_logging_once() {
#ifdef USE_SPDLOG
    static bool inited = false;
    if(!inited){
      // use colored console sink
      auto logger = spdlog::stdout_color_mt("factorlib");
      spdlog::set_default_logger(logger);

      // 根据编译/裁剪宏设置日志级别
#if defined(FACTORLIB_NO_DEBUG_TRACE)
      // 当编译期裁掉 TRACE/DEBUG 时，强制 INFO 级别
      spdlog::set_level(spdlog::level::info);
#elif defined(NDEBUG)
      spdlog::set_level(spdlog::level::info);  // Release模式
#else
      spdlog::set_level(spdlog::level::debug); // Debug模式
#endif

      spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
      inited = true;
    }
#endif
  }

} // namespace log
} // namespace factorlib

// 条件编译的日志宏
#ifdef USE_SPDLOG
  #if defined(FACTORLIB_NO_DEBUG_TRACE)
    // 编译期裁剪：移除 TRACE/DEBUG 宏（无论 Debug/Release）
    #define LOG_TRACE(...)
    #define LOG_DEBUG(...)
  #elif defined(NDEBUG)
    // Release模式：移除 TRACE/DEBUG 日志
    #define LOG_TRACE(...)
    #define LOG_DEBUG(...)
  #else
    // Debug模式：包含所有日志
    #define LOG_TRACE(...) do { ::factorlib::log::init_logging_once(); spdlog::trace(__VA_ARGS__); } while(0)
    #define LOG_DEBUG(...) do { ::factorlib::log::init_logging_once(); spdlog::debug(__VA_ARGS__); } while(0)
  #endif
  // INFO/WARN/ERROR在所有模式下都保留
  #define LOG_INFO(...)  do { ::factorlib::log::init_logging_once(); spdlog::info(__VA_ARGS__); } while(0)
  #define LOG_WARN(...)  do { ::factorlib::log::init_logging_once(); spdlog::warn(__VA_ARGS__); } while(0)
  #define LOG_ERROR(...) do { ::factorlib::log::init_logging_once(); spdlog::error(__VA_ARGS__); } while(0)
#else
  inline void __flog_fallback(const char* level, const char* fmt, ...) {
    std::fprintf(stderr, "[%s] ", level);
    va_list ap; va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fprintf(stderr, "\n");
    std::fflush(stderr);
  }

  #ifdef NDEBUG
    // Release模式：移除TRACE和DEBUG日志
    #define LOG_TRACE(...)
    #define LOG_DEBUG(...)
  #else
    // Debug模式：包含所有日志
    #define LOG_TRACE(...) __flog_fallback("TRACE", __VA_ARGS__)
    #define LOG_DEBUG(...) __flog_fallback("DEBUG", __VA_ARGS__)
  #endif
  // INFO/WARN/ERROR在所有模式下都保留
  #define LOG_INFO(...)  __flog_fallback("INFO",  __VA_ARGS__)
  #define LOG_WARN(...)  __flog_fallback("WARN",  __VA_ARGS__)
  #define LOG_ERROR(...) __flog_fallback("ERROR", __VA_ARGS__)
#endif
