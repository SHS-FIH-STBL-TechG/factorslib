
#include "utils/log.h"
#include <mutex>
#include <cstdarg>
#include <cstdio>

/**
 * @file log.cpp
 * @brief 日志实现：线程安全地打印到标准错误输出。
 */

namespace factorlib {

/**
 * @brief 实现日志打印，线程安全（互斥锁保护）
 */
void log_message(LogLevel lvl, const char* funcSig, const char* fmt, ...) {
    static std::mutex m;
    std::lock_guard<std::mutex> lk(m);
    const char* lvlStr = (lvl==LogLevel::INFO ? "INFO" : (lvl==LogLevel::WARN ? "WARN":"ERROR"));
    std::fprintf(stderr, "[%s] [%s] ", lvlStr, funcSig);
    va_list ap; va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fputc('\n', stderr);
}

} // namespace factorlib
