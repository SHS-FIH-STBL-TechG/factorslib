#include "tools/kline_csv_loader.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace factorlib::tools {

// ---------------------------------------------------------------------
// KlineCsvLoader：读取 tests/data 或指定目录下的日频 CSV，并转换为
// factorlib::Bar 序列，支持 code 过滤与起止时间裁剪。
// ---------------------------------------------------------------------

namespace {

// 工具函数：将日期 (UTC) 转换为毫秒时间戳，默认选择收盘 15:00
int64_t make_time_ms(int year, int month, int day, int hour = 15) {
    std::tm tm{};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = 0;
    tm.tm_sec = 0;
#if defined(_WIN32)
    return static_cast<int64_t>(_mkgmtime(&tm)) * 1000;
#else
    return static_cast<int64_t>(timegm(&tm)) * 1000;
#endif
}

} // namespace

KlineCsvLoader::KlineCsvLoader(std::string data_dir)
    : _root_dir(std::move(data_dir)) {}

std::vector<KlineCsvSeries> KlineCsvLoader::load(
    const std::vector<std::string>& codes_filter,
    std::optional<int64_t> start_time_ms,
    std::optional<int64_t> end_time_ms) const {
    namespace fs = std::filesystem;

    std::vector<KlineCsvSeries> result;
    fs::path root(_root_dir);
    if (!fs::exists(root) || !fs::is_directory(root)) {
        throw std::runtime_error("数据目录不存在: " + root.string());
    }

    // 枚举目录下的所有 CSV 文件
    for (const auto& entry : fs::directory_iterator(root)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".csv") {
            continue;
        }
        const auto filename = entry.path().filename().string();
        const auto code = extract_code(filename);
        if (!accept_code(codes_filter, code)) {
            continue;
        }

        std::ifstream ifs(entry.path());
        if (!ifs.is_open()) {
            throw std::runtime_error("无法打开 CSV 文件: " + entry.path().string());
        }

        KlineCsvSeries series;
        series.table_name = filename;
        series.code = code;

        std::string line;
        bool header_skipped = false;
        while (std::getline(ifs, line)) {
            if (!header_skipped) {
                header_skipped = true;
                continue;
            }
            if (line.empty()) continue;
            auto cells = parse_csv_line(line);
            if (cells.empty()) continue;

            auto ts_opt = parse_date_to_ms(cells.front());
            if (!ts_opt.has_value()) continue;
            int64_t ts = *ts_opt;
            if (start_time_ms && ts < *start_time_ms) {
                continue;
            }
            if (end_time_ms && ts > *end_time_ms) {
                continue;
            }

            auto open = cells.size() > 1 ? parse_number(cells[1]) : std::nullopt;
            auto high = cells.size() > 2 ? parse_number(cells[2]) : std::nullopt;
            auto low  = cells.size() > 3 ? parse_number(cells[3]) : std::nullopt;
            auto close = cells.size() > 4 ? parse_number(cells[4]) : std::nullopt;
            auto volume = cells.size() > 9 ? parse_number(cells[9]) : std::nullopt;
            auto turnover = cells.size() > 10 ? parse_number(cells[10]) : std::nullopt;
            if (!close.has_value()) continue;

            factorlib::Bar bar{};
            bar.instrument_id = code;
            bar.data_time_ms = ts;
            bar.open = open.value_or(*close);
            bar.high = high.value_or(*close);
            bar.low  = low.value_or(*close);
            bar.close = *close;
            bar.volume = volume ? static_cast<uint64_t>(*volume) : 0;
            bar.turnover = turnover.value_or(0.0);
            bar.interval_ms = 24 * 60 * 60 * 1000;
            series.bars.push_back(bar);
        }

        std::sort(series.bars.begin(), series.bars.end(),
                  [](const factorlib::Bar& a, const factorlib::Bar& b) {
                      return a.data_time_ms < b.data_time_ms;
                  });

        if (!series.bars.empty()) {
            result.emplace_back(std::move(series));
        }
    }

    std::sort(result.begin(), result.end(),
              [](const KlineCsvSeries& lhs, const KlineCsvSeries& rhs) {
                  return lhs.code < rhs.code;
              });
    return result;
}

// 处理无引号嵌套的简单 CSV 行
std::vector<std::string> KlineCsvLoader::parse_csv_line(const std::string& line) {
    std::vector<std::string> cells;
    std::string current;
    bool in_quotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        char ch = line[i];
        if (ch == '"') {
            if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
                current.push_back('"');
                ++i;
            } else {
                in_quotes = !in_quotes;
            }
        } else if (ch == ',' && !in_quotes) {
            cells.push_back(trim(current));
            current.clear();
        } else {
            current.push_back(ch);
        }
    }
    cells.push_back(trim(current));
    return cells;
}

// 去除字符串两端的空白
std::string KlineCsvLoader::trim(const std::string& value) {
    auto begin = value.begin();
    while (begin != value.end() && std::isspace(static_cast<unsigned char>(*begin))) {
        ++begin;
    }
    auto end = value.end();
    while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
        --end;
    }
    return std::string(begin, end);
}

// 将 yyyy-mm-dd 转成 UTC 毫秒时间戳
std::optional<int64_t> KlineCsvLoader::parse_date_to_ms(const std::string& date_token) {
    int y = 0, m = 0, d = 0;
    if (std::sscanf(date_token.c_str(), "%d-%d-%d", &y, &m, &d) == 3) {
        return make_time_ms(y, m, d);
    }
    return std::nullopt;
}

// 解析数字，自动去掉千分位和空格
std::optional<double> KlineCsvLoader::parse_number(const std::string& token) {
    if (token.empty()) return std::nullopt;
    std::string sanitized;
    sanitized.reserve(token.size());
    for (char ch : token) {
        if (ch == ',' || std::isspace(static_cast<unsigned char>(ch))) continue;
        sanitized.push_back(ch);
    }
    try {
        return std::stod(sanitized);
    } catch (...) {
        return std::nullopt;
    }
}

// CSV 文件名形如 CODE-其他信息.csv，提取 CODE 作为标的
std::string KlineCsvLoader::extract_code(const std::string& file_name) {
    auto stem = file_name;
    auto pos = stem.find('-');
    if (pos != std::string::npos) {
        stem = stem.substr(0, pos);
    } else if (auto dot = stem.rfind('.'); dot != std::string::npos) {
        stem = stem.substr(0, dot);
    }
    return stem;
}

// 根据命令行传入的过滤列表判断是否需要加载该标的
bool KlineCsvLoader::accept_code(const std::vector<std::string>& filters,
                                 const std::string& code) {
    if (filters.empty()) return true;
    return std::find(filters.begin(), filters.end(), code) != filters.end();
}

} // namespace factorlib::tools
