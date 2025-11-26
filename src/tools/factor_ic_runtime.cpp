// src/tools/factor_ic_runtime.cpp
// ---------------------------------------------------------------------------
// 运行时 IC 统计器：
//  - 由 DataBus::register_topic/publish 钩子自动注册所有 double 主题；
//  - 通过 ic_runtime_on_publish 记录每个 (topic, scoped_code) 的输出历史；
//  - 通过 ic_runtime_ingest_bars/bridge 调用得到每日 Bar，计算 forward return；
//  - 最终提供 per-code 的多主题 IC 报告，scope 维度与 ScopeKey 对齐（code|w）。
// ---------------------------------------------------------------------------
#include "tools/factor_ic_runtime.h"

#if FACTORLIB_ENABLE_IC_RUNTIME

#include <cmath>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include "utils/databus.h"
#include "stat_factors/memory_kernel_decay_factor.h"

namespace factorlib::tools {

namespace {

struct CodeState {
    ScopeKey scope;
    std::string table_name;
    std::vector<TableFactorSample> samples;
    std::optional<std::pair<int64_t, double>> latest_factor;
    std::unordered_map<int64_t, double> pending_values;
};

struct TopicState {
    std::string name;
    std::unordered_map<std::string, CodeState> codes; // key = scoped_code
};

struct ForwardState {
    bool has_last = false;
    factorlib::Bar last_bar{};
};

struct Binding {
    std::string topic;
    std::string scoped_code;
};

std::mutex g_mutex;
std::unordered_map<std::string, TopicState> g_topics;
std::unordered_map<std::string, ForwardState> g_forward_state; // base code -> last bar
std::unordered_map<std::string, std::vector<Binding>> g_code_bindings; // base code -> topics
std::unordered_set<std::string> g_binding_keys; // topic + '\n' + scoped_code
std::unordered_map<std::string, std::string> g_code_labels; // base code -> table label

std::string make_binding_key(const std::string& topic, const std::string& scoped_code) {
    return topic + '\n' + scoped_code;
}

void ensure_binding_locked(const std::string& topic,
                           const ScopeKey& scope,
                           const std::string& scoped_code) {
    auto key = make_binding_key(topic, scoped_code);
    if (g_binding_keys.insert(key).second) {
        g_code_bindings[scope.code].push_back(Binding{topic, scoped_code});
    }
}

void apply_label_locked(const std::string& base_code, const std::string& label) {
    if (label.empty()) return;
    g_code_labels[base_code] = label;
    auto it = g_code_bindings.find(base_code);
    if (it == g_code_bindings.end()) return;
    for (const auto& binding : it->second) {
        auto topic_it = g_topics.find(binding.topic);
        if (topic_it == g_topics.end()) continue;
        auto code_it = topic_it->second.codes.find(binding.scoped_code);
        if (code_it == topic_it->second.codes.end()) continue;
        code_it->second.table_name = label;
    }
}

void dispatch_forward_locked(const std::string& base_code,
                             int64_t ts_ms,
                             double forward_return) {
    auto bindings_it = g_code_bindings.find(base_code);
    if (bindings_it == g_code_bindings.end()) return;
    for (const auto& binding : bindings_it->second) {
        auto topic_it = g_topics.find(binding.topic);
        if (topic_it == g_topics.end()) continue;
        auto code_it = topic_it->second.codes.find(binding.scoped_code);
        if (code_it == topic_it->second.codes.end()) continue;

        auto& state = code_it->second;
        auto val_it = state.pending_values.find(ts_ms);
        if (val_it == state.pending_values.end()) continue;

        if (state.table_name.empty()) {
            if (auto lbl = g_code_labels.find(base_code); lbl != g_code_labels.end()) {
                state.table_name = lbl->second;
            }
        }

        state.samples.push_back(TableFactorSample{
            ts_ms,
            val_it->second,
            forward_return
        });
        state.pending_values.erase(val_it);
    }
}

} // namespace

void ic_runtime_clear() {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_topics.clear();
    g_forward_state.clear();
    g_code_bindings.clear();
    g_binding_keys.clear();
    g_code_labels.clear();
}

void ic_runtime_register_topic(const std::string& topic) {
    if (topic.empty()) return;
    std::lock_guard<std::mutex> lk(g_mutex);
    g_topics.emplace(topic, TopicState{topic, {}});
}

void ic_runtime_bind_label(const std::string& code, const std::string& table_label) {
    if (code.empty() || table_label.empty()) return;
    std::lock_guard<std::mutex> lk(g_mutex);
    apply_label_locked(code, table_label);
}

void ic_runtime_on_publish(const std::string& topic,
                           const std::string& scoped_code,
                           int64_t ts_ms,
                           double value) {
    if (topic.empty() || scoped_code.empty() || !std::isfinite(value)) return;
    std::lock_guard<std::mutex> lk(g_mutex);
    auto& topic_state = g_topics[topic];
    topic_state.name = topic;
    auto& code_state = topic_state.codes[scoped_code];
    if (code_state.scope.code.empty()) {
        code_state.scope = parse_scope_code(scoped_code);
        ensure_binding_locked(topic, code_state.scope, scoped_code);
        if (auto lbl = g_code_labels.find(code_state.scope.code); lbl != g_code_labels.end()) {
            code_state.table_name = lbl->second;
        }
    }
    code_state.pending_values[ts_ms] = value;
    code_state.latest_factor = std::make_pair(ts_ms, value);
}

void ic_runtime_ingest_bars(const std::vector<factorlib::Bar>& bars,
                            const std::string& table_hint) {
    if (bars.empty()) return;
    std::lock_guard<std::mutex> lk(g_mutex);
    for (const auto& bar : bars) {
        if (bar.instrument_id.empty() || bar.close <= 0.0) continue;
        const std::string code = bar.instrument_id;

        if (!table_hint.empty()) {
            apply_label_locked(code, table_hint);
        } else if (auto it = g_code_labels.find(code); it != g_code_labels.end()) {
            apply_label_locked(code, it->second);
        }

        auto& forward = g_forward_state[code];
        if (forward.has_last) {
            const auto& prev = forward.last_bar;
            if (prev.close > 0.0) {
                double forward_return = bar.close / prev.close - 1.0;
                dispatch_forward_locked(code, prev.data_time_ms, forward_return);
            }
        }
        forward.last_bar = bar;
        forward.has_last = true;
    }
}

std::vector<TableIcReport> ic_runtime_snapshot_reports() {
    std::lock_guard<std::mutex> lk(g_mutex);
    std::unordered_map<std::string, TableIcReport> aggregated;
    for (const auto& [topic_name, topic_state] : g_topics) {
        for (const auto& [scoped_code, state] : topic_state.codes) {
            if (state.samples.empty()) continue;
            const std::string base_code = state.scope.code.empty() ? scoped_code : state.scope.code;
            auto& report = aggregated[base_code];
            report.code = base_code;
            if (report.table_name.empty()) {
                if (!state.table_name.empty()) {
                    report.table_name = state.table_name;
                } else if (auto lbl = g_code_labels.find(base_code); lbl != g_code_labels.end()) {
                    report.table_name = lbl->second;
                } else {
                    report.table_name = base_code;
                }
            }
            TopicIcSeries series;
            series.topic = topic_name;
            series.scope = state.scope.code.empty() ? ScopeKey{base_code, 0} : state.scope;
            series.samples = state.samples;
            series.latest_factor = state.latest_factor;
            report.topics.push_back(std::move(series));
        }
    }

    std::vector<TableIcReport> reports;
    reports.reserve(aggregated.size());
    for (auto& [_, report] : aggregated) {
        reports.push_back(std::move(report));
    }
    return reports;
}

bool ic_runtime_has_samples() {
    std::lock_guard<std::mutex> lk(g_mutex);
    for (const auto& [_, topic_state] : g_topics) {
        for (const auto& [__, state] : topic_state.codes) {
            if (!state.samples.empty()) return true;
        }
    }
    return false;
}

} // namespace factorlib::tools

#endif // FACTORLIB_ENABLE_IC_RUNTIME
