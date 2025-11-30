// include/config/config_utils.h
#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <limits>

#include "config/runtime_config.h"

namespace factorlib::config {

inline std::string trim_copy(const std::string& s) {
    size_t l = 0, r = s.size();
    while (l < r && std::isspace(static_cast<unsigned char>(s[l]))) ++l;
    while (r > l && std::isspace(static_cast<unsigned char>(s[r - 1]))) --r;
    return s.substr(l, r - l);
}

/// @brief 解析逗号分隔的整数列表（<=0 的值被忽略）
///        允许用户在 INI 中写 “30, 60 ,120”，该函数统一裁剪空格并去重。
inline std::vector<int> parse_int_list(const std::string& csv) {
    std::vector<int> values;
    std::stringstream ss(csv);
    std::string token;
    while (std::getline(ss, token, ',')) {
        auto trimmed = trim_copy(token);
        if (trimmed.empty()) continue;
        try {
            int v = std::stoi(trimmed);
            if (v > 0) values.push_back(v);
        } catch (...) {
            // 忽略非法条目
        }
    }
    if (values.empty()) return values;
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
    return values;
}

/**
 * @brief 从运行配置加载窗口数组：优先读取 “section.window_sizes” 列表，
 *        若未配置则退回单一的 “section.window_size”。
 *        这样每个因子可以无缝支持“多窗口 + 后向兼容单窗口”。
 */
inline std::vector<int> load_window_sizes(const std::string& section,
                                          int fallback_window) {
    auto& cfg = RuntimeConfig::instance();
    int single = cfg.geti(section + ".window_size", fallback_window);
    if (single <= 0) single = fallback_window;
    static const std::string empty_string;
    auto multi_raw = cfg.get(section + ".window_sizes", empty_string);
    auto list = parse_int_list(multi_raw);
    if (list.empty()) list.push_back(single);
    return list;
}

/// @brief 解析 section 里的 time_frequencies=... 配置；留空则返回空数组，表示使用全局频率。
inline int64_t unit_to_ms(const std::string& unit_raw) {
    auto unit = unit_raw;
    std::transform(unit.begin(), unit.end(), unit.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    auto match = [&](const char* s){ return unit == s; };

    constexpr int64_t kMsPerSecond = 1000LL;
    constexpr int64_t kMsPerMinute = 60LL * kMsPerSecond;
    constexpr int64_t kMsPerHour   = 60LL * kMsPerMinute;
    constexpr int64_t kMsPerDay    = 24LL * kMsPerHour;
    constexpr int64_t kMsPerWeek   = 7LL  * kMsPerDay;
    constexpr int64_t kMsPerMonth  = 30LL * kMsPerDay;
    constexpr int64_t kMsPerQuarter= 3LL  * kMsPerMonth;
    constexpr int64_t kMsPerYear   = 365LL* kMsPerDay;

    if (match("ms") || match("millisecond") || unit == "毫秒") return 1;
    if (match("s")  || match("sec") || match("second") || unit == "秒") return kMsPerSecond;
    if (match("m")  || match("min") || match("minute") || unit == "分钟") return kMsPerMinute;
    if (match("h")  || match("hr")  || match("hour")   || unit == "小时") return kMsPerHour;
    if (match("d")  || match("day") || unit == "日") return kMsPerDay;
    if (match("w")  || match("week")|| unit == "周") return kMsPerWeek;
    if (match("month") || unit == "月") return kMsPerMonth;
    if (match("q")  || match("quarter") || unit == "季度") return kMsPerQuarter;
    if (match("y")  || match("year") || unit == "年") return kMsPerYear;
    return -1;
}

inline bool parse_frequency_token(const std::string& token, int64_t& out_ms) {
    auto trimmed = trim_copy(token);
    if (trimmed.empty()) return false;
    size_t idx = 0;
    while (idx < trimmed.size() && (std::isdigit(static_cast<unsigned char>(trimmed[idx])) || trimmed[idx]=='.')) {
        ++idx;
    }
    double multiplier = 1.0;
    if (idx > 0) {
        try {
            multiplier = std::stod(trimmed.substr(0, idx));
        } catch (...) {
            return false;
        }
    }
    std::string unit = trim_copy(trimmed.substr(idx));
    if (unit.empty()) unit = "ms";
    auto base_ms = unit_to_ms(unit);
    if (base_ms <= 0) return false;
    double value = multiplier * static_cast<double>(base_ms);
    if (!(value > 0.0)) return false;
    constexpr int64_t kMaxMs = 10LL * 365LL * 24LL * 60LL * 60LL * 1000LL; // 最长不超过 10 年
    if (value > static_cast<double>(kMaxMs)) value = static_cast<double>(kMaxMs);
    out_ms = static_cast<int64_t>(value);
    if (out_ms < 1) out_ms = 1;
    return true;
}

inline std::vector<int64_t> load_time_frequencies(const std::string& section) {
    static const std::string empty_string;
    auto raw = RuntimeConfig::instance().get(section + ".time_frequencies", empty_string);
    std::vector<int64_t> freqs;
    std::stringstream ss(raw);
    std::string token;
    while (std::getline(ss, token, ',')) {
        int64_t value = 0;
        if (parse_frequency_token(token, value)) {
            freqs.push_back(value);
        }
    }
    if (freqs.empty()) return freqs;
    std::sort(freqs.begin(), freqs.end());
    freqs.erase(std::unique(freqs.begin(), freqs.end()), freqs.end());
    return freqs;
}

} // namespace factorlib::config
