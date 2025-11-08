#pragma once
/**
 * Simple logging facade for factorlib.
 * If USE_SPDLOG is defined and headers are available, map to spdlog.
 * Otherwise fall back to fprintf(stderr, ...).
 */
 
#ifdef USE_SPDLOG
  #include <spdlog/spdlog.h>
  #include <spdlog/sinks/stdout_color_sinks.h>
#endif
#include <cstdio>
#include <cstdarg>

namespace factorlib {
namespace log {

// initialize logging once per process (no-op for fallback)
inline void init_logging_once() {
#ifdef USE_SPDLOG
    static bool inited = false;
    if(!inited){
        // use colored console sink
        auto logger = spdlog::stdout_color_mt("factorlib");
        spdlog::set_default_logger(logger);
        spdlog::set_level(spdlog::level::trace);
        spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
        inited = true;
    }
#endif
}

} // namespace log
} // namespace factorlib

#ifdef USE_SPDLOG
  #define LOG_TRACE(...) do { ::factorlib::log::init_logging_once(); spdlog::trace(__VA_ARGS__); } while(0)
  #define LOG_DEBUG(...) do { ::factorlib::log::init_logging_once(); spdlog::debug(__VA_ARGS__); } while(0)
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
  #define LOG_TRACE(...) __flog_fallback("TRACE", __VA_ARGS__)
  #define LOG_DEBUG(...) __flog_fallback("DEBUG", __VA_ARGS__)
  #define LOG_INFO(...)  __flog_fallback("INFO",  __VA_ARGS__)
  #define LOG_WARN(...)  __flog_fallback("WARN",  __VA_ARGS__)
  #define LOG_ERROR(...) __flog_fallback("ERROR", __VA_ARGS__)
#endif
