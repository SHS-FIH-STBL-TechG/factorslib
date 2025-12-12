#pragma once

#include "core/types.h"

#include <optional>
#include <string>
#include <vector>

namespace factorlib::tools {

struct KlineCsvSeries {
    std::string table_name;
    std::string code;
    std::vector<factorlib::Bar> bars;
};

// 与 src/tools 版本一致：读取 tests/data 或指定目录下的日频 CSV。
class KlineCsvLoader {
public:
    explicit KlineCsvLoader(std::string data_dir);

    std::vector<KlineCsvSeries> load(const std::vector<std::string>& codes_filter,
                                     std::optional<int64_t> start_time_ms,
                                     std::optional<int64_t> end_time_ms) const;

private:
    std::string _root_dir;

    static std::vector<std::string> parse_csv_line(const std::string& line);
    static std::string trim(const std::string& value);
    static std::optional<int64_t> parse_date_to_ms(const std::string& date_token);
    static std::optional<double> parse_number(const std::string& token);
    static std::string extract_code(const std::string& file_name);
    static bool accept_code(const std::vector<std::string>& filters, const std::string& code);
};

} // namespace factorlib::tools

