// tests/factor_ic_tool.cpp
// ---------------------------------------------------------------------------
// 以 MemoryKernelDecayFactor 为例的 IC 评估辅助：
//  1. 读取 tests/data 目录下的行情统计 CSV，每张表对应一个 code；
//  2. 将 CSV 行转换成 factorlib::Bar，通过 bridge/ingress 走真实流程下发因子；
//  3. 因子产出的 memory_kernel/decay 结果由 tools/factor_ic_runtime 在 DataBus 上监听；
//  4. runtime 在收到下一期 Bar 时计算 forward return，形成 (因子, 下一期收益) 样本并输出 IC。
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <functional>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <limits>

#include "bridge/ingress.h"
#include "stat_factors/memory_kernel_decay_factor.h"
#include "stat_factors/pca_angular_momentum_factor.h"
#include "stat_factors/pca_structure_stability_factor.h"
#include "utils/databus.h"
#include "utils/scope_key.h"
#include "utils/types.h"
#include "utils/trace_helper.h"
#include "tools/factor_ic_runtime.h"

namespace factorlib::tools {

namespace fs = std::filesystem;

/**
 * @brief 透传外部推送的上交所快照（SH）；数据无需再次落盘，可直接进入 ingress。
 */
inline void ingest_snapshot_sh(const std::vector<std_SnapshotStockSH>& rows) {
    factorlib::bridge::ingest_snapshot_sh(rows);
}

/**
 * @brief 透传外部推送的深交所快照（SZ），未来可在本工具内直接复用。
 */
inline void ingest_snapshot_sz(const std::vector<std_SnapshotStockSZ>& rows) {
    factorlib::bridge::ingest_snapshot_sz(rows);
}

/**
 * @brief 透传外部推送的逐笔（合表 ONT）数据，供 IC 计算链路后续扩展。
 */
inline void ingest_ont(const std::vector<std_OrdAndExeInfo>& rows) {
    factorlib::bridge::ingest_ont(rows);
}

// ---- 日期 / CSV 解析辅助 ----

struct DateKey {
    int year{};
    int month{};
    int day{};
};

inline std::time_t make_utc_time(int year, int month, int day, int hour = 15) {
    std::tm tm{};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
#if defined(_WIN32)
    return _mkgmtime(&tm);
#else
    return timegm(&tm);
#endif
}

inline DateKey to_date(std::time_t ts) {
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &ts);
#else
    gmtime_r(&ts, &tm);
#endif
    return {tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday};
}

inline std::string format_date(const DateKey& d) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d", d.year, d.month, d.day);
    return {buf};
}

inline std::string format_date_ms(int64_t ts_ms) {
    const auto seconds = static_cast<std::time_t>(ts_ms / 1000);
    return format_date(to_date(seconds));
}

inline std::optional<DateKey> parse_date(const std::string& token) {
    int y = 0, m = 0, d = 0;
    if (std::sscanf(token.c_str(), "%d-%d-%d", &y, &m, &d) == 3) {
        return DateKey{y, m, d};
    }
    return std::nullopt;
}

inline void strip_utf8_bom(std::string& line) {
    if (line.size() >= 3 &&
        static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF) {
        line.erase(0, 3);
    }
}

inline std::string trim(std::string value) {
    auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

inline std::string remove_thousands(std::string value) {
    value.erase(std::remove(value.begin(), value.end(), ','), value.end());
    value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());
    return value;
}

inline std::optional<double> parse_number(const std::string& token) {
    if (token.empty()) return std::nullopt;
    try {
        return std::stod(remove_thousands(token));
    } catch (...) {
        return std::nullopt;
    }
}

inline std::vector<std::string> parse_csv_line(const std::string& line) {
    std::vector<std::string> cells;
    std::string current;
    bool in_quotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
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

/**
 * @brief CSV 解析结果：保留“原始表名 + 对应 code + 排好序的 Bar 序列”。
 */
struct AssetSeries {
    std::string table_name;
    std::string code;
    std::vector<factorlib::Bar> bars;
};

// ---- CSV 加载与预处理 ----

/**
 * @brief 负责遍历 tests/data 下所有 CSV，并转换为 AssetSeries。
 *        解析时顺便补齐开高低和数值清洗，保证后续 ingress 无需额外处理。
 */
class CsvBarLoader {
public:
    explicit CsvBarLoader(fs::path root) : root_dir_(std::move(root)) {}

    std::vector<AssetSeries> load_all() const {
        std::vector<AssetSeries> all;
        if (!fs::exists(root_dir_)) {
            throw std::runtime_error("数据目录不存在: " + root_dir_.string());
        }
        for (const auto& entry : fs::directory_iterator(root_dir_)) {
            if (!entry.is_regular_file() || entry.path().extension() != ".csv") continue;
            auto series = load_file(entry.path());
            if (!series.bars.empty()) {
                all.emplace_back(std::move(series));
            }
        }
        if (all.empty()) {
            throw std::runtime_error("未找到任何 CSV：请确认 tests/data 下存在行情统计文件。");
        }
        return all;
    }

private:
    fs::path root_dir_;

    static std::string extract_code(const fs::path& file) {
        std::string stem = file.stem().string();
        auto dash_pos = stem.find('-');
        if (dash_pos != std::string::npos) {
            stem = stem.substr(0, dash_pos);
        }
        return stem;
    }

    static int64_t date_to_ms(const DateKey& date) {
        auto ts = make_utc_time(date.year, date.month, date.day, 15);
        return static_cast<int64_t>(ts) * 1000;
    }

    AssetSeries load_file(const fs::path& path) const {
        AssetSeries series;
        series.table_name = path.filename().string();
        series.code = extract_code(path);

        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            throw std::runtime_error("无法打开 CSV 文件: " + path.string());
        }
        std::string line;
        bool header_skipped = false;
        while (std::getline(ifs, line)) {
            if (!header_skipped) {
                strip_utf8_bom(line);
                header_skipped = true;
                continue;
            }
            if (line.empty()) continue;
            auto cells = parse_csv_line(line);
            if (cells.empty()) continue;

            auto date = parse_date(cells[0]);
            auto close = cells.size() > 4 ? parse_number(cells[4]) : std::nullopt;
            if (!date || !close) continue;

            auto open = cells.size() > 1 ? parse_number(cells[1]) : std::nullopt;
            auto high = cells.size() > 2 ? parse_number(cells[2]) : std::nullopt;
            auto low  = cells.size() > 3 ? parse_number(cells[3]) : std::nullopt;
            auto turnover = cells.size() > 10 ? parse_number(cells[10]) : std::nullopt;
            auto volume   = cells.size() > 9  ? parse_number(cells[9])  : std::nullopt;

            factorlib::Bar bar{};
            bar.instrument_id = series.code;
            bar.data_time_ms  = date_to_ms(*date);
            bar.open          = open.value_or(*close);
            bar.high          = high.value_or(*close);
            bar.low           = low.value_or(*close);
            bar.close         = *close;
            bar.turnover      = turnover.value_or(0.0);
            bar.volume        = volume ? static_cast<uint64_t>(*volume * 100) : 0;
            bar.interval_ms   = 24 * 60 * 60 * 1000; // 日频
            series.bars.push_back(bar);
        }

        std::sort(series.bars.begin(), series.bars.end(),
                  [](const factorlib::Bar& lhs, const factorlib::Bar& rhs) {
                      return lhs.data_time_ms < rhs.data_time_ms;
                  });
        return series;
    }
};

// ---- IC 计算 ----

inline double compute_correlation(const std::vector<TableFactorSample>& samples) {
    const auto n = samples.size();
    if (n < 2) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto& s : samples) {
        mean_x += s.factor;
        mean_y += s.forward;
    }
    mean_x /= static_cast<double>(n);
    mean_y /= static_cast<double>(n);

    double cov = 0.0;
    double var_x = 0.0;
    double var_y = 0.0;
    for (const auto& s : samples) {
        const double dx = s.factor - mean_x;
        const double dy = s.forward - mean_y;
        cov += dx * dy;
        var_x += dx * dx;
        var_y += dy * dy;
    }
    if (var_x <= 0.0 || var_y <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return cov / std::sqrt(var_x * var_y);
}

struct IcSeriesMetrics {
    double ic{};
    double ratio_abs_gt_0_1{};
    double ratio_abs_gt_0_5{};
    double ratio_abs_gt_0_9{};
};

IcSeriesMetrics compute_ic_metrics(const std::vector<TableFactorSample>& samples) {
    IcSeriesMetrics metrics;
    metrics.ic = compute_correlation(samples);

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_x2 = 0.0;
    double sum_y2 = 0.0;
    double sum_xy = 0.0;
    std::size_t count = 0;
    std::size_t valid_history = 0;
    std::size_t gt_0_1 = 0;
    std::size_t gt_0_5 = 0;
    std::size_t gt_0_9 = 0;

    for (const auto& sample : samples) {
        if (!std::isfinite(sample.factor) || !std::isfinite(sample.forward)) continue;
        ++count;
        sum_x += sample.factor;
        sum_y += sample.forward;
        sum_x2 += sample.factor * sample.factor;
        sum_y2 += sample.forward * sample.forward;
        sum_xy += sample.factor * sample.forward;
        if (count < 2) continue;
        double mean_x = sum_x / static_cast<double>(count);
        double mean_y = sum_y / static_cast<double>(count);
        double var_x = sum_x2 - static_cast<double>(count) * mean_x * mean_x;
        double var_y = sum_y2 - static_cast<double>(count) * mean_y * mean_y;
        if (var_x <= 0.0 || var_y <= 0.0) continue;
        double cov = sum_xy - static_cast<double>(count) * mean_x * mean_y;
        double corr = cov / std::sqrt(var_x * var_y);
        if (!std::isfinite(corr)) continue;
        ++valid_history;
        const double abs_corr = std::abs(corr);
        if (abs_corr > 0.1) ++gt_0_1;
        if (abs_corr > 0.5) ++gt_0_5;
        if (abs_corr > 0.9) ++gt_0_9;
    }

    if (valid_history > 0) {
        const double denom = static_cast<double>(valid_history);
        metrics.ratio_abs_gt_0_1 = static_cast<double>(gt_0_1) / denom;
        metrics.ratio_abs_gt_0_5 = static_cast<double>(gt_0_5) / denom;
        metrics.ratio_abs_gt_0_9 = static_cast<double>(gt_0_9) / denom;
    }
    return metrics;
}

/**
 * @brief orchestrator：完成“加载 → 下发 → 取数 → 统计 → 打印”整条链路。
 *
 * - CSV 只负责提供 K 线，真实场景可直接改为调用 ingest_snapshot/ingest_ont；
 * - DataBus 用于承接 MemoryKernelDecayFactor 的输出；
 * - run() 仅输出 per-table 时间序列 IC，聚焦“因子是否可用”。
 */
class FactorIcRunner {
public:
    FactorIcRunner()
        : loader_(fs::path("tests") / "data") {}

    /**
     * @brief CLI 入口：按照“加载 → 初始化因子 → 走 ingress → 汇总 IC”的顺序执行。
     */
    int run() {
        auto series = loader_.load_all();
        if (series.empty()) {
            std::cerr << "[错误] 没有可用的行情数据。" << std::endl;
            return 1;
        }

        std::vector<std::string> codes;
        codes.reserve(series.size());
        for (const auto& s : series) {
            codes.push_back(s.code);
        }

        DataBus::instance().reset();
        MemoryKernelDecayFactor::register_topics(topic_capacity_);
        PcaAngularMomentumFactor::register_topics(topic_capacity_);
        PcaStructureStabilityFactor::register_topics(topic_capacity_);

        MemoryKernelConfig cfg;
        cfg.window_size = 64;
        cfg.L = 24;
        cfg.alpha = 0.7;

        auto mem_factor = std::make_shared<MemoryKernelDecayFactor>(codes, cfg);
        PcaAngularMomentumConfig pca_cfg;
        auto pca_factor = std::make_shared<PcaAngularMomentumFactor>(codes, pca_cfg);

        PcaStructureStabilityConfig stab_cfg;
        auto stab_factor = std::make_shared<PcaStructureStabilityFactor>(codes, stab_cfg);

        bridge::set_factors({mem_factor, pca_factor, stab_factor});

        factorlib::tools::ic_runtime_clear();

        for (const auto& asset : series) {
            factorlib::tools::ic_runtime_bind_label(asset.code, asset.table_name);
            bridge::ingest_kline(asset.bars); // 走 ingress，以保持与生产一致的触发顺序
        }

        auto table_reports = factorlib::tools::ic_runtime_snapshot_reports();
        for (auto& report : table_reports) {
            if (report.table_name.empty()) {
                report.table_name = report.code;
            }
        }
        if (table_reports.empty()) {
            if (!factorlib::tools::ic_runtime_enabled()) {
                std::cout << "[提示] 当前编译未启用 IC 计算 (FACTORLIB_ENABLE_IC_RUNTIME=OFF)。" << std::endl;
            } else {
                std::cout << "[提示] 未得到任何 per-table 样本，可能因子尚未就绪。" << std::endl;
            }
            return 0;
        }
        print_table_reports(table_reports);
        return 0;
    }

private:
    CsvBarLoader loader_;
    static constexpr std::size_t topic_capacity_ = 8'192;

    /**
     * @brief 输出“==== IC 统计 [表名] ====”，并展示该表的时间序列指标。
     */
    void print_table_reports(const std::vector<TableIcReport>& reports) const {
        if (reports.empty()) return;
        std::cout << "\n";
        for (const auto& report : reports) {
            std::cout << "====  IC 统计 ["
                      << report.table_name << "] ====" << std::endl;
            std::cout << "  代码: " << report.code << std::endl;
            if (report.topics.empty()) {
                std::cout << "  暂无可用因子样本。" << std::endl << std::endl;
                continue;
            }
            for (const auto& series : report.topics) {
                const std::size_t sample_cnt = series.samples.size();
                std::cout << "  因子: " << series.topic;
                if (series.scope.window > 0) {
                    std::cout << " (window=" << series.scope.window << ")";
                }
                std::cout << " | 样本数: " << sample_cnt << std::endl;
                auto metrics = compute_ic_metrics(series.samples);
                double ic = metrics.ic;
                if (sample_cnt < 2 || !std::isfinite(ic)) {
                    std::cout << "    时间序列 IC: 样本不足" << std::endl;
                } else {
                    std::cout << "    时间序列 IC: " << ic << std::endl;
                    if (metrics.ratio_abs_gt_0_1 > 0.1) {
                        std::cout << "    |IC|>0.1 占比: " << metrics.ratio_abs_gt_0_1 << std::endl;
                    }
                    if (metrics.ratio_abs_gt_0_5 > 0.1) {
                        std::cout << "    |IC|>0.5 占比: " << metrics.ratio_abs_gt_0_5 << std::endl;
                    }
                    if (metrics.ratio_abs_gt_0_9 > 0.1) {
                        std::cout << "    |IC|>0.9 占比: " << metrics.ratio_abs_gt_0_9 << std::endl;
                    }
                }
            }
            std::cout << std::endl;
        }
    }
};

} // namespace factorlib::tools

int main() {
    // 初始化 Perfetto 追踪
    bool trace_enabled = factorlib::trace::TraceHelper::initialize("factor_ic_tool.pftrace");
    if (trace_enabled) {
        std::cout << "[追踪] Perfetto 已启用，trace 将保存到: "
                  << factorlib::trace::TraceHelper::current_trace_path() << std::endl;
    }

    int result = 1;
    try {
        factorlib::tools::FactorIcRunner runner;
        result = runner.run();
    } catch (const std::exception& ex) {
        std::cerr << "[错误] 因子 IC 计算失败: " << ex.what() << std::endl;
    }

    // 关闭追踪并保存文件
    if (trace_enabled) {
        factorlib::trace::TraceHelper::shutdown();
        std::cout << "[追踪] Trace 已保存到: "
                  << factorlib::trace::TraceHelper::current_trace_path() << std::endl;
        std::cout << "        在 https://ui.perfetto.dev 查看可视化结果" << std::endl;
    }

    return result;
}
