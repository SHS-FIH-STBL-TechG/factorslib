// include/tools/factor_ic_runtime.h
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "core/types.h"
#include "core/scope_key.h"

namespace factorlib::tools {

struct TableFactorSample {
    int64_t ts_ms{};
    double factor{};
    double forward{};
};

struct TopicIcSeries {
    std::string topic;
    ScopeKey scope;
    std::vector<TableFactorSample> samples;
    std::optional<std::pair<int64_t, double>> latest_factor;
};

struct TableIcReport {
    std::string table_name;
    std::string code;
    std::vector<TopicIcSeries> topics;
};

#if FACTORLIB_ENABLE_IC_RUNTIME

void ic_runtime_clear();
void ic_runtime_ingest_bars(const std::vector<factorlib::Bar>& bars,
                            const std::string& table_hint = {});
void ic_runtime_bind_label(const std::string& code, const std::string& table_label);
void ic_runtime_register_topic(const std::string& topic);
void ic_runtime_on_publish(const std::string& topic,
                           const std::string& scoped_code,
                           int64_t ts_ms,
                           double value);
std::vector<TableIcReport> ic_runtime_snapshot_reports();
bool ic_runtime_has_samples();
constexpr bool ic_runtime_enabled() { return true; }

#else

inline void ic_runtime_clear() {}
inline void ic_runtime_ingest_bars(const std::vector<factorlib::Bar>&,
                                   const std::string& = {}) {}
inline void ic_runtime_bind_label(const std::string&, const std::string&) {}
inline void ic_runtime_register_topic(const std::string&) {}
inline void ic_runtime_on_publish(const std::string&, const std::string&,
                                  int64_t, double) {}
inline std::vector<TableIcReport> ic_runtime_snapshot_reports() { return {}; }
inline bool ic_runtime_has_samples() { return false; }
constexpr bool ic_runtime_enabled() { return false; }

#endif

} // namespace factorlib::tools
