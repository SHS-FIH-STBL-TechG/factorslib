// include/utils/trace_helper.h
#pragma once

#include <string>
#include <cstdint>

namespace factorlib {
namespace trace {

/**
 * @brief Perfetto 追踪辅助类
 *
 * 用于追踪因子计算流程，支持为每个因子的每个窗口生成唯一的追踪 ID。
 * 生成的 trace 可以在 https://ui.perfetto.dev 上可视化查看。
 */
class TraceHelper {
public:
    /**
     * @brief 初始化 Perfetto 追踪系统
     * @param output_path trace 文件输出路径（默认为 "factor_trace.pftrace"）
     * @return 是否初始化成功
     */
    static bool initialize(const std::string& output_path = "factor_trace.pftrace");

    /**
     * @brief 停止追踪并保存文件
     */
    static void shutdown();

    /**
     * @brief 判断追踪是否已启用
     */
    static bool is_enabled();

    /**
     * @brief 开始追踪一个作用域（数据入口或因子计算）
     *
     * @param category 追踪分类（如 "ingress", "factor_compute"）
     * @param name 追踪名称（如因子名称）
     * @param code 股票代码
     * @param window 窗口大小（0 表示无窗口）
     * @param unique_id 数据的唯一标识（如时间戳或序列号）
     * @return trace_id 用于标记追踪的唯一 ID
     */
    static uint64_t begin_scope(
        const char* category,
        const char* name,
        const std::string& code,
        int window,
        uint64_t unique_id
    );

    /**
     * @brief 结束追踪作用域
     * @param trace_id begin_scope 返回的追踪 ID
     */
    static void end_scope(uint64_t trace_id);

    /**
     * @brief 添加追踪事件（瞬时事件，不需要 end）
     *
     * @param category 追踪分类
     * @param name 事件名称
     * @param code 股票代码
     * @param window 窗口大小
     * @param unique_id 数据的唯一标识
     */
    static void trace_event(
        const char* category,
        const char* name,
        const std::string& code,
        int window,
        uint64_t unique_id
    );

    /**
     * @brief 添加追踪计数器（用于显示数值变化）
     *
     * @param category 追踪分类
     * @param name 计数器名称
     * @param code 股票代码
     * @param window 窗口大小
     * @param value 计数器值
     */
    static void trace_counter(
        const char* category,
        const char* name,
        const std::string& code,
        int window,
        double value
    );

private:
    static bool initialized_;
    static std::string output_path_;
};

/**
 * @brief RAII 风格的追踪作用域守卫
 *
 * 使用方法：
 * {
 *     TraceScope trace("factor_compute", "MyFactor", "000001.SZ", 60, timestamp);
 *     // ... 因子计算代码 ...
 * } // 自动结束追踪
 */
class TraceScope {
public:
    TraceScope(
        const char* category,
        const char* name,
        const std::string& code,
        int window,
        uint64_t unique_id
    ) : trace_id_(0) {
        if (TraceHelper::is_enabled()) {
            trace_id_ = TraceHelper::begin_scope(category, name, code, window, unique_id);
        }
    }

    ~TraceScope() {
        if (trace_id_ != 0) {
            TraceHelper::end_scope(trace_id_);
        }
    }

    // 禁止拷贝
    TraceScope(const TraceScope&) = delete;
    TraceScope& operator=(const TraceScope&) = delete;

private:
    uint64_t trace_id_;
};

// 便捷宏定义
#ifdef FACTORLIB_ENABLE_PERFETTO

#define TRACE_SCOPE(category, name, code, window, unique_id) \
    factorlib::trace::TraceScope _trace_scope_##__LINE__(category, name, code, window, unique_id)

#define TRACE_EVENT(category, name, code, window, unique_id) \
    factorlib::trace::TraceHelper::trace_event(category, name, code, window, unique_id)

#define TRACE_COUNTER(category, name, code, window, value) \
    factorlib::trace::TraceHelper::trace_counter(category, name, code, window, value)

#else

// 禁用追踪时为空操作
#define TRACE_SCOPE(category, name, code, window, unique_id) (void)0
#define TRACE_EVENT(category, name, code, window, unique_id) (void)0
#define TRACE_COUNTER(category, name, code, window, value) (void)0

#endif // FACTORLIB_ENABLE_PERFETTO

} // namespace trace
} // namespace factorlib
