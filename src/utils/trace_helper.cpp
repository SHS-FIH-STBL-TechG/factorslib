// src/utils/trace_helper.cpp
#include "utils/trace_helper.h"
#include "utils/log.h"

#ifdef FACTORLIB_ENABLE_PERFETTO
#include "perfetto.h"
#include <fstream>
#include <memory>
#include <thread>

// MSVC 在链接 TrackEvent 静态库时偶尔会遗漏 DataSourceHelper 的隐式实例化，
// 这里显式实例化一次以确保符号定义存在。
namespace perfetto {
template <>
internal::DataSourceType& DataSourceHelper<internal::TrackEventDataSource,
                                           internal::TrackEventDataSourceTraits>::type() {
    static internal::DataSourceType type{};
    return type;
}
} // namespace perfetto

// 定义 Perfetto 追踪分类
PERFETTO_DEFINE_CATEGORIES(
    perfetto::Category("ingress")
        .SetDescription("Data ingress events"),
    perfetto::Category("factor_compute")
        .SetDescription("Factor computation events"),
    perfetto::Category("factor_window")
        .SetDescription("Factor window processing"),
    perfetto::Category("databus")
        .SetDescription("DataBus publish/subscribe events")
);

PERFETTO_TRACK_EVENT_STATIC_STORAGE();

#endif // FACTORLIB_ENABLE_PERFETTO

namespace factorlib {
namespace trace {

bool TraceHelper::initialized_ = false;
std::string TraceHelper::output_path_;

#ifdef FACTORLIB_ENABLE_PERFETTO

namespace {
    std::unique_ptr<std::ofstream> g_trace_file;
    std::unique_ptr<perfetto::TracingSession> g_tracing_session;
}

bool TraceHelper::initialize(const std::string& output_path) {
    if (initialized_) {
        LOG_WARN("TraceHelper already initialized");
        return true;
    }

    output_path_ = output_path;

    // 配置 Perfetto
    perfetto::TracingInitArgs args;
    args.backends = perfetto::kInProcessBackend;
    perfetto::Tracing::Initialize(args);
    perfetto::TrackEvent::Register();

    // 创建追踪会话配置
    perfetto::TraceConfig cfg;
    cfg.add_buffers()->set_size_kb(102400);  // 100 MB buffer
    auto* ds_cfg = cfg.add_data_sources()->mutable_config();
    ds_cfg->set_name("track_event");

    // 启动追踪会话
    g_trace_file = std::make_unique<std::ofstream>(output_path, std::ios::binary);
    if (!g_trace_file->is_open()) {
        LOG_ERROR("Failed to open trace file: {}", output_path);
        return false;
    }

    g_tracing_session = perfetto::Tracing::NewTrace();
    g_tracing_session->Setup(cfg);
    g_tracing_session->StartBlocking();

    initialized_ = true;
    LOG_INFO("Perfetto tracing initialized, output: {}", output_path);
    return true;
}

void TraceHelper::shutdown() {
    if (!initialized_) {
        return;
    }

    if (g_tracing_session) {
        // 停止追踪
        g_tracing_session->StopBlocking();

        // 读取追踪数据
        std::vector<char> trace_data(g_tracing_session->ReadTraceBlocking());

        // 写入文件
        if (g_trace_file && g_trace_file->is_open()) {
            g_trace_file->write(trace_data.data(), trace_data.size());
            g_trace_file->close();
            LOG_INFO("Trace saved to: {}, size: {} bytes", output_path_, trace_data.size());
        }

        g_tracing_session.reset();
    }

    g_trace_file.reset();
    initialized_ = false;
}

bool TraceHelper::is_enabled() {
    return initialized_;
}

uint64_t TraceHelper::begin_scope(
    const char* category,
    const char* name,
    const std::string& code,
    int window,
    uint64_t unique_id
) {
    if (!initialized_) {
        return 0;
    }

    // 使用 Perfetto 的 TRACE_EVENT_BEGIN 宏
    perfetto::Track track(unique_id);

    // 根据 category 字符串选择对应的分类
    if (std::string(category) == "ingress") {
        TRACE_EVENT_BEGIN("ingress", perfetto::StaticString{name}, track, "code", code, "window", window);
    } else if (std::string(category) == "factor_compute") {
        TRACE_EVENT_BEGIN("factor_compute", perfetto::StaticString{name}, track, "code", code, "window", window);
    } else if (std::string(category) == "factor_window") {
        TRACE_EVENT_BEGIN("factor_window", perfetto::StaticString{name}, track, "code", code, "window", window);
    } else if (std::string(category) == "databus") {
        TRACE_EVENT_BEGIN("databus", perfetto::StaticString{name}, track, "code", code, "window", window);
    }

    return unique_id;
}

void TraceHelper::end_scope(uint64_t trace_id) {
    if (!initialized_ || trace_id == 0) {
        return;
    }

    perfetto::Track track(trace_id);
    // TRACE_EVENT_END 需要 category，但我们不知道原始的 category
    // 使用一个通用的 category
    TRACE_EVENT_END("ingress", track);
}

void TraceHelper::trace_event(
    const char* category,
    const char* name,
    const std::string& code,
    int window,
    uint64_t unique_id
) {
    if (!initialized_) {
        return;
    }

    perfetto::Track track(unique_id);

    // 根据 category 字符串选择对应的分类，使用 TRACE_EVENT_INSTANT
    if (std::string(category) == "ingress") {
        TRACE_EVENT_INSTANT("ingress", perfetto::StaticString{name}, track, "code", code, "window", window);
    } else if (std::string(category) == "factor_compute") {
        TRACE_EVENT_INSTANT("factor_compute", perfetto::StaticString{name}, track, "code", code, "window", window);
    } else if (std::string(category) == "factor_window") {
        TRACE_EVENT_INSTANT("factor_window", perfetto::StaticString{name}, track, "code", code, "window", window);
    } else if (std::string(category) == "databus") {
        TRACE_EVENT_INSTANT("databus", perfetto::StaticString{name}, track, "code", code, "window", window);
    }
}

void TraceHelper::trace_counter(
    const char* category,
    const char* name,
    const std::string& code,
    int window,
    double value
) {
    if (!initialized_) {
        return;
    }

    // 为每个 code+window 组合创建唯一的 counter track
    uint64_t track_id = std::hash<std::string>{}(code + "_" + std::to_string(window));
    perfetto::CounterTrack counter_track(perfetto::DynamicString{name}, track_id);

    // 根据 category 字符串选择对应的分类
    if (std::string(category) == "ingress") {
        TRACE_COUNTER("ingress", counter_track, value);
    } else if (std::string(category) == "factor_compute") {
        TRACE_COUNTER("factor_compute", counter_track, value);
    } else if (std::string(category) == "factor_window") {
        TRACE_COUNTER("factor_window", counter_track, value);
    } else if (std::string(category) == "databus") {
        TRACE_COUNTER("databus", counter_track, value);
    }
}

#else // !FACTORLIB_ENABLE_PERFETTO

// 没有启用 Perfetto 时的空实现
bool TraceHelper::initialize(const std::string& output_path) {
    output_path_ = output_path;
    LOG_INFO("Perfetto tracing is disabled (FACTORLIB_ENABLE_PERFETTO not defined)");
    return false;
}

void TraceHelper::shutdown() {
    // Do nothing
}

bool TraceHelper::is_enabled() {
    return false;
}

uint64_t TraceHelper::begin_scope(
    const char* /*category*/,
    const char* /*name*/,
    const std::string& /*code*/,
    int /*window*/,
    uint64_t /*unique_id*/
) {
    return 0;
}

void TraceHelper::end_scope(uint64_t /*trace_id*/) {
    // Do nothing
}

void TraceHelper::trace_event(
    const char* /*category*/,
    const char* /*name*/,
    const std::string& /*code*/,
    int /*window*/,
    uint64_t /*unique_id*/
) {
    // Do nothing
}

void TraceHelper::trace_counter(
    const char* /*category*/,
    const char* /*name*/,
    const std::string& /*code*/,
    int /*window*/,
    double /*value*/
) {
    // Do nothing
}

#endif // FACTORLIB_ENABLE_PERFETTO

} // namespace trace
} // namespace factorlib
