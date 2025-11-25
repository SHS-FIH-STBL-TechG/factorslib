// tests/factor_ic_tool.cpp
// ---------------------------------------------------------------------------
// 该文件实现了一个独立的“因子 IC 评估”命令行工具：
// 1. 读取 tests/data 目录下的日频 CSV 行情（默认包含开高低收、成交量等字段）
// 2. 根据配置聚合出日/周/月/季度/年等频率的收盘价序列
// 3. 构造一个示例性的滚动收益因子（可替换成任意真实因子输出）
// 4. 计算每日截面的 IC 序列与统计值（均值、标准差、IR 等）
// 5. 将结果输出到 tests/output/ic_report_*.csv，并打印到 stdout
//
// 如需扩展到真实的 stat_factors，可在 BuildObservations 或 Evaluate 中
// 将“因子值”替换为实际因子时间序列，本工具的 IC 输出逻辑无需变动。
// ---------------------------------------------------------------------------

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace factorlib {
namespace ic {

namespace fs = std::filesystem;

constexpr int64_t kSecondsPerDay = 24 * 60 * 60;

inline std::time_t MakeUtcTime(int year, int month, int day) {
    std::tm tm{};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = 12;
#if defined(_WIN32)
    return _mkgmtime(&tm);
#else
    return timegm(&tm);
#endif
}

inline std::tm ToTm(std::time_t time) {
    std::tm out{};
#if defined(_WIN32)
    gmtime_s(&out, &time);
#else
    gmtime_r(&time, &out);
#endif
    return out;
}

// DateKey 表示一个 UTC 日期（不含时间），用于作为各类 map 的键值。
struct DateKey {
    int year{};
    int month{};
    int day{};

    bool operator<(const DateKey& other) const {
        return std::tie(year, month, day) < std::tie(other.year, other.month, other.day);
    }

    bool operator==(const DateKey& other) const {
        return year == other.year && month == other.month && day == other.day;
    }

    std::time_t to_time_t() const { return MakeUtcTime(year, month, day); }
};

inline DateKey FromTime(std::time_t t) {
    auto tm = ToTm(t);
    return {tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday};
}

// 以天为单位移动日期（用于求周/月等周期的起止时间）
inline DateKey ShiftDays(const DateKey& date, int delta) {
    auto t = date.to_time_t() + static_cast<int64_t>(delta) * kSecondsPerDay;
    return FromTime(t);
}

inline std::string FormatDate(const DateKey& d) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d", d.year, d.month, d.day);
    return std::string(buf);
}

enum class Frequency {
    kMillisecond,
    kSecond,
    kMinute,
    kHour,
    kDaily,
    kWeekly,
    kMonthly,
    kQuarterly,
    kYearly
};

// 将 Frequency 枚举映射成友好的中文标签，便于日志打印
inline std::string FrequencyName(Frequency f) {
    switch (f) {
        case Frequency::kMillisecond: return "毫秒";
        case Frequency::kSecond: return "秒";
        case Frequency::kMinute: return "分钟";
        case Frequency::kHour: return "小时";
        case Frequency::kDaily: return "日";
        case Frequency::kWeekly: return "周";
        case Frequency::kMonthly: return "月";
        case Frequency::kQuarterly: return "季度";
        case Frequency::kYearly: return "年";
        default: return "未知";
    }
}

// FactorEvaluationConfig 控制工具的整体行为：数据来源、输出目录、
// 需要执行的频率/窗口、以及是否保留 snapshot/ONT 等占位接口。
struct FactorEvaluationConfig {
    fs::path data_root = fs::path("tests") / "data";
    fs::path output_root = fs::path("tests") / "output";
    std::vector<Frequency> slow_frequencies = {
        Frequency::kDaily,
        Frequency::kWeekly,
        Frequency::kMonthly,
        Frequency::kQuarterly,
        Frequency::kYearly
    };
    std::vector<Frequency> realtime_frequencies = {
        Frequency::kMillisecond,
        Frequency::kSecond,
        Frequency::kMinute,
        Frequency::kHour
    };
    std::vector<std::size_t> windows = {30, 300};
    bool keep_snapshot_tables = true;
    bool keep_ont_tables = true;
};

struct BarRecord {
    DateKey date;
    double open{};
    double high{};
    double low{};
    double close{};
};

struct AggregatedPoint {
    DateKey period_start;
    DateKey period_end;
    double close{};
};

struct FactorObservation {
    DateKey date;
    double factor{};
    double forward_return{};
    std::string symbol;
};

struct IcEntry {
    DateKey date;
    double value{};
    std::size_t sample_size{};
};

struct IcStats {
    std::vector<IcEntry> entries;
    double mean{};
    double stddev{};
    double ir{};
    double positive_ratio{};
    double abs_mean{};
    double t_stat{};
};

// CSV 中的数字自带逗号或空格，此处统一去除后再做 stod。
inline std::string RemoveThousandsSeparators(std::string value) {
    value.erase(std::remove(value.begin(), value.end(), ','), value.end());
    value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());
    return value;
}

// 容错解析一个字段为 double，失败时返回 nullopt。
inline std::optional<double> ParseNumber(const std::string& text) {
    if (text.empty()) return std::nullopt;
    std::string sanitized = RemoveThousandsSeparators(text);
    try {
        return std::stod(sanitized);
    } catch (...) {
        return std::nullopt;
    }
}

inline std::optional<DateKey> ParseDate(const std::string& text) {
    int y = 0, m = 0, d = 0;
    if (std::sscanf(text.c_str(), "%d-%d-%d", &y, &m, &d) == 3) {
        return DateKey{y, m, d};
    }
    return std::nullopt;
}

// 有些 Excel 导出的 CSV 会在首行携带 UTF-8 BOM，需要去除。
inline void StripUtf8Bom(std::string& line) {
    if (line.size() >= 3 &&
        static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF) {
        line.erase(0, 3);
    }
}

// 简单的首尾空白裁剪，便于解析 CSV 单元格。
inline std::string Trim(std::string value) {
    auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

// 解析一行 CSV，支持双引号转义。仅保留我们关心的 12 个字段。
inline std::vector<std::string> ParseCsvLine(const std::string& line) {
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
            cells.push_back(Trim(current));
            current.clear();
        } else {
            current.push_back(ch);
        }
    }
    cells.push_back(Trim(current));
    return cells;
}

// DailyCsvLoader 负责扫描 tests/data 下的所有 CSV，并转成统一的 BarRecord 序列。
class DailyCsvLoader {
public:
    explicit DailyCsvLoader(fs::path root) : root_dir_(std::move(root)) {}

    struct AssetSeries {
        std::string symbol;
        std::vector<BarRecord> bars;
    };

    std::vector<AssetSeries> LoadAll() const {
        std::vector<AssetSeries> all;
        if (!fs::exists(root_dir_)) {
            throw std::runtime_error("数据目录不存在: " + root_dir_.string());
        }
        // 逐个遍历目录下的 CSV 文件，文件名前缀作为“合约/指数”标识。
        for (const auto& entry : fs::directory_iterator(root_dir_)) {
            if (!entry.is_regular_file()) continue;
            if (entry.path().extension() != ".csv") continue;
            AssetSeries series = LoadFile(entry.path());
            if (!series.bars.empty()) {
                all.emplace_back(std::move(series));
            }
        }
        if (all.empty()) {
            throw std::runtime_error("未在 " + root_dir_.string() + " 中找到可用的日频K线 .csv 文件。");
        }
        return all;
    }

private:
    fs::path root_dir_;

    static std::string ExtractSymbol(const fs::path& file) {
        std::string stem = file.stem().string();
        auto dash_pos = stem.find('-');
        if (dash_pos != std::string::npos) {
            stem = stem.substr(0, dash_pos);
        }
        return stem;
    }

    AssetSeries LoadFile(const fs::path& path) const {
        AssetSeries asset;
        asset.symbol = ExtractSymbol(path);
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            throw std::runtime_error("无法打开 CSV 文件: " + path.string());
        }
        std::string line;
        bool header_skipped = false;
        while (std::getline(ifs, line)) {
            if (!header_skipped) {
                StripUtf8Bom(line);
                header_skipped = true;
                continue;
            }
            if (line.empty()) continue;
            auto cells = ParseCsvLine(line);
            if (cells.empty()) continue;
            auto date_opt = ParseDate(cells[0]);
            auto close_opt = cells.size() > 4 ? ParseNumber(cells[4]) : std::nullopt;
            if (!date_opt || !close_opt) continue;
            auto open_opt = cells.size() > 1 ? ParseNumber(cells[1]) : std::nullopt;
            auto high_opt = cells.size() > 2 ? ParseNumber(cells[2]) : std::nullopt;
            auto low_opt = cells.size() > 3 ? ParseNumber(cells[3]) : std::nullopt;
            BarRecord record{
                *date_opt,
                open_opt.value_or(*close_opt),
                high_opt.value_or(*close_opt),
                low_opt.value_or(*close_opt),
                *close_opt
            };
            asset.bars.emplace_back(record);
        }

        std::sort(asset.bars.begin(), asset.bars.end(),
                  [](const BarRecord& lhs, const BarRecord& rhs) { return lhs.date < rhs.date; });
        return asset;
    }
};

inline DateKey PeriodAnchor(const DateKey& date, Frequency freq) {
    auto tm = ToTm(date.to_time_t());
    switch (freq) {
        case Frequency::kWeekly: {
            int wday = tm.tm_wday; // 0 = Sunday
            int days_back = (wday + 6) % 7; // Monday as start
            return ShiftDays(date, -days_back);
        }
        case Frequency::kMonthly:
            return DateKey{date.year, date.month, 1};
        case Frequency::kQuarterly: {
            int quarter = (date.month - 1) / 3;
            int month = quarter * 3 + 1;
            return DateKey{date.year, month, 1};
        }
        case Frequency::kYearly:
            return DateKey{date.year, 1, 1};
        case Frequency::kDaily:
        default:
            return date;
    }
}

// 将日频 Bar 序列聚合成目标频率（周、月、季度、年度等）的末值。
std::vector<AggregatedPoint> Resample(const std::vector<BarRecord>& bars, Frequency freq) {
    if (freq == Frequency::kDaily) {
        std::vector<AggregatedPoint> daily;
        daily.reserve(bars.size());
        for (const auto& bar : bars) {
            daily.push_back({bar.date, bar.date, bar.close});
        }
        return daily;
    }

    std::map<DateKey, AggregatedPoint, std::less<>> buckets;
    for (const auto& bar : bars) {
        DateKey anchor = PeriodAnchor(bar.date, freq);
        auto& bucket = buckets[anchor];
        if (bucket.period_start.year == 0) {
            bucket.period_start = anchor;
        }
        bucket.period_end = bar.date;
        bucket.close = bar.close;
    }

    std::vector<AggregatedPoint> aggregated;
    aggregated.reserve(buckets.size());
    for (const auto& item : buckets) {
        aggregated.push_back(item.second);
    }
    std::sort(aggregated.begin(), aggregated.end(),
              [](const AggregatedPoint& lhs, const AggregatedPoint& rhs) {
                  return lhs.period_end < rhs.period_end;
              });
    return aggregated;
}

// 根据聚合后的序列构造“因子值 + 下一期收益”的样本。默认因子为
//   factor = price(t) / price(t-window) - 1
//   forward = price(t+1) / price(t) - 1
// 若要接入真实因子，可只保留 forward_return 的计算，并将 factor 改为外部输入。
std::vector<FactorObservation> BuildObservations(const std::vector<AggregatedPoint>& series,
                                                 std::size_t window,
                                                 const std::string& symbol) {
    std::vector<FactorObservation> observations;
    if (series.size() <= window + 1) {
        return observations;
    }
    for (std::size_t idx = window; idx + 1 < series.size(); ++idx) {
        const auto& current = series[idx];
        const auto& base = series[idx - window];
        const auto& next = series[idx + 1];
        if (base.close == 0.0 || current.close == 0.0) continue;
        double factor = current.close / base.close - 1.0;
        double forward = next.close / current.close - 1.0;
        observations.push_back({current.period_end, factor, forward, symbol});
    }
    return observations;
}

// 计算单期截面的皮尔逊相关系数（IC）。为鲁棒起见需至少 2 只标的。
double ComputeCorrelation(const std::vector<FactorObservation>& data) {
    const std::size_t n = data.size();
    if (n < 2) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto& item : data) {
        mean_x += item.factor;
        mean_y += item.forward_return;
    }
    mean_x /= static_cast<double>(n);
    mean_y /= static_cast<double>(n);

    double cov = 0.0;
    double var_x = 0.0;
    double var_y = 0.0;
    for (const auto& item : data) {
        double dx = item.factor - mean_x;
        double dy = item.forward_return - mean_y;
        cov += dx * dy;
        var_x += dx * dx;
        var_y += dy * dy;
    }
    if (var_x <= 0.0 || var_y <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return cov / std::sqrt(var_x * var_y);
}

// 在 IC 序列的基础上，统计均值/标准差/IR/正值比例/|IC| 均值/t 值。
IcStats BuildStats(std::vector<IcEntry> entries) {
    IcStats stats;
    stats.entries = std::move(entries);
    const auto n = stats.entries.size();
    if (n == 0) return stats;

    double sum = 0.0;
    double abs_sum = 0.0;
    std::size_t positive = 0;
    for (const auto& entry : stats.entries) {
        sum += entry.value;
        abs_sum += std::abs(entry.value);
        if (entry.value > 0.0) positive++;
    }
    stats.mean = sum / static_cast<double>(n);
    stats.abs_mean = abs_sum / static_cast<double>(n);
    stats.positive_ratio = static_cast<double>(positive) / static_cast<double>(n);

    if (n > 1) {
        double sq_sum = 0.0;
        for (const auto& entry : stats.entries) {
            double diff = entry.value - stats.mean;
            sq_sum += diff * diff;
        }
        stats.stddev = std::sqrt(sq_sum / static_cast<double>(n - 1));
    }
    if (stats.stddev > 0.0) {
        stats.ir = stats.mean / stats.stddev;
        stats.t_stat = stats.mean / (stats.stddev / std::sqrt(static_cast<double>(n)));
    }
    return stats;
}

// MarketMicroStructureHooks 只是展示“保留快照/逐笔接口”的提示，
// 方便后续在该工具中接入更高频的数据源。
class MarketMicroStructureHooks {
public:
    MarketMicroStructureHooks(bool enable_snapshot,
                              bool enable_ont,
                              std::vector<Frequency> realtime_freqs)
        : enable_snapshot_(enable_snapshot),
          enable_ont_(enable_ont),
          realtime_freqs_(std::move(realtime_freqs)) {}

    void DescribeInterfaces() const {
        if (enable_snapshot_) {
            std::cout << "[信息] 已保留快照数据接口（snapshot_quotes_csv.csv）。" << std::endl;
        }
        if (enable_ont_) {
            std::cout << "[信息] 已保留逐笔成交/委托接口（ONT 数据表）。" << std::endl;
        }
        if (!realtime_freqs_.empty()) {
            std::cout << "[信息] 高频计算频率配置：";
            bool first = true;
            for (auto freq : realtime_freqs_) {
                if (!first) std::cout << "、";
                std::cout << FrequencyName(freq);
                first = false;
            }
            std::cout << "（暂未接入数据，但接口保持可扩展）。" << std::endl;
        }
    }

private:
    bool enable_snapshot_;
    bool enable_ont_;
    std::vector<Frequency> realtime_freqs_;
};

// FactorIcTool 作为整个程序的 orchestrator：调用 loader → 计算 → 输出。
class FactorIcTool {
public:
    explicit FactorIcTool(FactorEvaluationConfig cfg)
        : config_(std::move(cfg)),
          loader_(config_.data_root),
          micro_structure_hooks_(config_.keep_snapshot_tables,
                                 config_.keep_ont_tables,
                                 config_.realtime_frequencies) {}

    int Run() {
        micro_structure_hooks_.DescribeInterfaces();
        const auto assets = loader_.LoadAll();
        fs::create_directories(config_.output_root);

        for (auto freq : config_.slow_frequencies) {
            for (auto window : config_.windows) {
                Evaluate(assets, freq, window);
            }
        }
        return 0;
    }

private:
    FactorEvaluationConfig config_;
    DailyCsvLoader loader_;
    MarketMicroStructureHooks micro_structure_hooks_;

    static std::string CsvFileName(Frequency freq, std::size_t window) {
        std::ostringstream oss;
        oss << "ic_report_" << FrequencyName(freq) << "_window" << window << ".csv";
        return oss.str();
    }

    // 输出时间序列结果，方便在 Excel/Notebook 中进一步分析。
    void WriteCsv(const fs::path& path, const IcStats& stats) const {
        std::ofstream ofs(path);
        ofs << "date,sample_size,ic\n";
        for (const auto& entry : stats.entries) {
            ofs << FormatDate(entry.date) << "," << entry.sample_size << "," << entry.value << "\n";
        }
    }

    // 在控制台打印一份概览，便于快速对比不同频率/窗口的表现。
    void PrintSummary(Frequency freq, std::size_t window, const IcStats& stats) const {
        std::cout << "==== 频率: " << FrequencyName(freq)
                  << " | 窗口: " << window << " ====" << std::endl;
        std::cout << "  样本数: " << stats.entries.size() << std::endl;
        std::cout << "  IC 均值: " << stats.mean << std::endl;
        std::cout << "  IC 标准差: " << stats.stddev << std::endl;
        std::cout << "  IC IR: " << stats.ir << std::endl;
        std::cout << "  正 IC 比例: " << stats.positive_ratio << std::endl;
        std::cout << "  |IC| 均值: " << stats.abs_mean << std::endl;
        std::cout << "  IC 均值 t 统计量: " << stats.t_stat << std::endl;
        if (!stats.entries.empty()) {
            std::cout << "  首尾截面示例: "
                      << FormatDate(stats.entries.front().date) << " => " << stats.entries.front().value
                      << " ，"
                      << FormatDate(stats.entries.back().date) << " => " << stats.entries.back().value
                      << std::endl;
        }
    }

    void Evaluate(const std::vector<DailyCsvLoader::AssetSeries>& assets,
                  Frequency freq,
                  std::size_t window) const {
        std::map<DateKey, std::vector<FactorObservation>, std::less<>> buckets;
        std::size_t available_assets = 0;
        for (const auto& asset : assets) {
            auto resampled = Resample(asset.bars, freq);
            auto obs = BuildObservations(resampled, window, asset.symbol);
            if (!obs.empty()) {
                available_assets++;
            }
            for (const auto& item : obs) {
                buckets[item.date].push_back(item);
            }
        }
        if (available_assets == 0) {
            std::cout << "[警告] 频率 " << FrequencyName(freq)
                      << " ，窗口 " << window << " 无可用资产历史。" << std::endl;
            return;
        }

        // 将同一日期的所有标的聚合成一个截面，计算该日 IC。
        std::vector<IcEntry> ic_entries;
        for (auto& pair : buckets) {
            if (pair.second.size() < 2) continue;
            double ic = ComputeCorrelation(pair.second);
            if (!std::isfinite(ic)) continue;
            ic_entries.push_back({pair.first, ic, pair.second.size()});
        }

        auto stats = BuildStats(std::move(ic_entries));
        if (stats.entries.empty()) {
            std::cout << "[警告] 频率 " << FrequencyName(freq)
                      << " ，窗口 " << window << " 无法形成有效截面（资产数量不足）。" << std::endl;
            return;
        }

        const auto csv_path = config_.output_root / CsvFileName(freq, window);
        WriteCsv(csv_path, stats);
        PrintSummary(freq, window, stats);
    }
};

} // namespace ic
} // namespace factorlib

int main() {
    try {
        factorlib::ic::FactorEvaluationConfig config;
        factorlib::ic::FactorIcTool tool(config);
        return tool.Run();
    } catch (const std::exception& ex) {
        std::cerr << "[错误] 因子 IC 计算失败: " << ex.what() << std::endl;
        return 1;
    }
}
