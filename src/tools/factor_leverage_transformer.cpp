#include "tools/factor_leverage_transformer.h"
#include "tools/kline_csv_loader.h"

#include "bridge/ingress.h"
#include "core/databus.h"
#include "factors/kline/low_freq_return_factor.h"
#include "utils/fs_compat.h"
#include "utils/log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace fs = std::filesystem;

namespace factorlib::tools {

// ---------------------------------------------------------------------
// CLI 工具入口：从 CSV 回放 K 线，驱动 LowFreqReturnFactor，并把结果
// 送入 FactorLeverageTransformer，最终输出 CSV + 分布图。
// ---------------------------------------------------------------------

struct CliOptions {
    std::string data_dir = "tests/data";
    std::string output_dir = "output/factor_leverage";
    std::optional<int64_t> start_ms;
    std::optional<int64_t> end_ms;
    std::vector<std::string> codes;
    std::size_t leverage_window = SlidingGaussianLeverage::kDefaultWindow;
    std::size_t topic_capacity = 64'000;
};

namespace {

// 采样点结构：方便统一持久化 code/时间/数值
struct SamplePoint {
    std::string code;
    int64_t timestamp;
    double value;
};

// 汇总器：订阅 DataBus 后，把原始因子与杠杆值分别存到向量
struct Collector {
    void add_factor(const std::string& code, int64_t ts, double value) {
        std::lock_guard<std::mutex> lk(mtx);
        factor_points.push_back({code, ts, value});
    }
    void add_leverage(const std::string& code, int64_t ts, double value) {
        std::lock_guard<std::mutex> lk(mtx);
        leverage_points.push_back({code, ts, value});
    }
    std::vector<SamplePoint> factor_points;
    std::vector<SamplePoint> leverage_points;
    std::mutex mtx;
};

std::optional<int64_t> parse_date(const std::string& arg) {
    if (arg.empty()) return std::nullopt;
    int y = 0, m = 0, d = 0;
    if (std::sscanf(arg.c_str(), "%d-%d-%d", &y, &m, &d) != 3) {
        return std::nullopt;
    }
    std::tm tm{};
    tm.tm_year = y - 1900;
    tm.tm_mon = m - 1;
    tm.tm_mday = d;
    tm.tm_hour = 0;
    tm.tm_min = 0;
    tm.tm_sec = 0;
#if defined(_WIN32)
    return static_cast<int64_t>(_mkgmtime(&tm)) * 1000;
#else
    return static_cast<int64_t>(timegm(&tm)) * 1000;
#endif
}

CliOptions parse_args_internal(int argc, char** argv) {
    CliOptions opts;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next_value = [&]() -> std::optional<std::string> {
            if (i + 1 >= argc) return std::nullopt;
            return std::string(argv[++i]);
        };
        if (arg == "--data-dir") {
            if (auto v = next_value()) opts.data_dir = *v;
        } else if (arg == "--output-dir") {
            if (auto v = next_value()) opts.output_dir = *v;
        } else if (arg == "--start-date") {
            if (auto v = next_value()) opts.start_ms = parse_date(*v);
        } else if (arg == "--end-date") {
            if (auto v = next_value()) {
                if (auto parsed = parse_date(*v)) {
                    // 包含当天结束 23:59:59
                    opts.end_ms = *parsed + (24ll * 60 * 60 * 1000) - 1;
                }
            }
        } else if (arg == "--codes") {
            if (auto v = next_value()) {
                std::stringstream ss(*v);
                std::string token;
                opts.codes.clear();
                while (std::getline(ss, token, ',')) {
                    if (!token.empty()) opts.codes.push_back(token);
                }
            }
        } else if (arg == "--window") {
            if (auto v = next_value()) {
                opts.leverage_window = std::max<std::size_t>(1, std::stoull(*v));
            }
        } else if (arg == "--topic-capacity") {
            if (auto v = next_value()) {
                opts.topic_capacity = std::max<std::size_t>(1024, std::stoull(*v));
            }
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "用法: factor_leverage_tool [--data-dir DIR] [--output-dir DIR]\n"
                         "                    [--codes CODE1,CODE2] [--start-date YYYY-MM-DD]\n"
                         "                    [--end-date YYYY-MM-DD] [--window N]\n";
            std::exit(0);
        }
    }
    return opts;
}

std::string format_time(int64_t ts_ms) {
    std::time_t seconds = static_cast<std::time_t>(ts_ms / 1000);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &seconds);
#else
    gmtime_r(&seconds, &tm);
#endif
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min);
    return std::string(buf);
}

// 将采样点写入 CSV，便于复盘与绘图
void write_csv(const fs::path& path, const std::vector<SamplePoint>& points) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw std::runtime_error("无法写入文件: " + path.string());
    }
    ofs << "code,timestamp,value\n";
    auto sorted = points;
    std::sort(sorted.begin(), sorted.end(),
              [](const SamplePoint& a, const SamplePoint& b) {
                  if (a.timestamp == b.timestamp) {
                      return a.code < b.code;
                  }
                  return a.timestamp < b.timestamp;
              });
    for (const auto& p : sorted) {
        ofs << p.code << ','
            << format_time(p.timestamp) << ','
            << std::setprecision(15) << p.value << '\n';
    }
}

void ensure_directory(const fs::path& dir) {
    if (!fs::exists(dir)) {
        fs::create_directories(dir);
    }
}

// 生成临时 Python 脚本并调用 matplotlib 输出直方图
bool run_plot_script(const fs::path& script_path,
                     const fs::path& csv_path,
                     const fs::path& png_path,
                     const std::string& title) {
    std::ofstream script(script_path);
    script << R"(import csv, sys, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def read_values(csv_path):
    values = []
    with open(csv_path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                v = float(row["value"])
                if math.isfinite(v):
                    values.append(v)
            except Exception:
                pass
    return values

def main():
    if len(sys.argv) < 4:
        print("Usage: python plot_hist.py data.csv output.png title")
        return 1
    csv_path, png_path, title = sys.argv[1], sys.argv[2], sys.argv[3]
    values = read_values(csv_path)
    if not values:
        print(f"[WARN] no values found in {csv_path}")
        return 0
    plt.figure(figsize=(8, 4.5))
    plt.hist(values, bins=50, color="#1f77b4", alpha=0.85, edgecolor="white")
    plt.title(title)
    plt.xlabel("value")
    plt.ylabel("frequency")
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(png_path)
    return 0

if __name__ == "__main__":
    sys.exit(main())
)";
    script.close();

    std::ostringstream cmd;
    cmd << "python \"" << script_path.string() << "\" \""
        << csv_path.string() << "\" \"" << png_path.string() << "\" \""
        << title << "\"";
    int rc = std::system(cmd.str().c_str());
    return rc == 0;
}

// 按时间排序所有 Bar，保证按真实时间驱动低频因子
std::vector<factorlib::Bar> collect_bars(
    const std::vector<KlineCsvSeries>& series_list) {
    std::vector<factorlib::Bar> bars;
    for (const auto& series : series_list) {
        bars.insert(bars.end(), series.bars.begin(), series.bars.end());
    }
    std::sort(bars.begin(), bars.end(),
              [](const factorlib::Bar& a, const factorlib::Bar& b) {
                  if (a.data_time_ms == b.data_time_ms) {
                      return a.instrument_id < b.instrument_id;
                  }
                  return a.data_time_ms < b.data_time_ms;
              });
    return bars;
}

// 逐条调用 ingress，模拟生产环境的因子驱动流程
void drive_factors(const std::vector<factorlib::Bar>& bars) {
    for (const auto& bar : bars) {
        factorlib::bridge::ingest_kline(std::vector<factorlib::Bar>{bar});
    }
}

std::vector<std::string> gather_codes(const std::vector<KlineCsvSeries>& series_list) {
    std::vector<std::string> codes;
    for (const auto& series : series_list) {
        codes.push_back(series.code);
    }
    std::sort(codes.begin(), codes.end());
    codes.erase(std::unique(codes.begin(), codes.end()), codes.end());
    return codes;
}

} // namespace

CliOptions parse_cli_options(int argc, char** argv) {
    return parse_args_internal(argc, argv);
}

int run_tool(const CliOptions& opts) {
    KlineCsvLoader loader(opts.data_dir);
    auto series_list = loader.load(opts.codes, opts.start_ms, opts.end_ms);
    if (series_list.empty()) {
        std::cerr << "[ERROR] 在目录 " << opts.data_dir << " 中找不到匹配的 CSV。" << std::endl;
        return 1;
    }

    auto codes = gather_codes(series_list);
    std::cout << "[INFO] 处理代码数量: " << codes.size() << std::endl;

    DataBus::instance().reset();
    LowFreqReturnFactor::register_topics(opts.topic_capacity);
    auto factor = std::make_shared<LowFreqReturnFactor>(codes);
    bridge::set_factors({factor});

    FactorLeverageSpec spec;
    spec.input_topic = "kline/ret_lowfreq_mu10";
    spec.output_topic = "kline/ret_lowfreq_mu10_leverage";
    spec.window = opts.leverage_window;
    FactorLeverageTransformer transformer({spec}, codes, opts.topic_capacity);
    transformer.start();

    Collector collector;
    for (const auto& code : codes) {
        // 订阅原始因子，用于输出“未高斯化”的分布图
        DataBus::instance().subscribe<double>(
            spec.input_topic,
            code,
            [&collector, code](const std::string&, int64_t ts, const double& value) {
                if (std::isfinite(value)) {
                    collector.add_factor(code, ts, value);
                }
            });
        // 订阅杠杆 topic，记录高斯化后的值
        DataBus::instance().subscribe<double>(
            spec.output_topic,
            code,
            [&collector, code](const std::string&, int64_t ts, const double& value) {
                if (std::isfinite(value)) {
                    collector.add_leverage(code, ts, value);
                }
            });
    }

    auto bars = collect_bars(series_list);
    std::cout << "[INFO] 推送 Bar 数量: " << bars.size() << std::endl;
    drive_factors(bars);
    transformer.stop();

    fs::path output_dir(opts.output_dir);
    ensure_directory(output_dir);
    fs::path factor_csv = output_dir / "factor_values.csv";
    fs::path leverage_csv = output_dir / "leverage_values.csv";
    write_csv(factor_csv, collector.factor_points);
    write_csv(leverage_csv, collector.leverage_points);
    std::cout << "[INFO] 已写入 " << factor_csv << " 和 " << leverage_csv << std::endl;

    fs::path script_path = output_dir / "plot_hist.py";
    fs::path factor_png = output_dir / "factor_distribution.png";
    fs::path leverage_png = output_dir / "leverage_distribution.png";
    bool factor_ok = run_plot_script(script_path, factor_csv, factor_png, "Factor Value Distribution");
    bool leverage_ok = run_plot_script(script_path, leverage_csv, leverage_png, "Leverage Value Distribution");
    if (!factor_ok || !leverage_ok) {
        std::cerr << "[WARN] 图像绘制失败，请确认已安装 Python 及 matplotlib。" << std::endl;
        return factor_ok && leverage_ok ? 0 : 2;
    }
    std::cout << "[INFO] 图像输出: " << factor_png << " , " << leverage_png << std::endl;
    return 0;
}

} // namespace factorlib::tools

int main(int argc, char** argv) {
    auto opts = factorlib::tools::parse_cli_options(argc, argv);
    try {
        return factorlib::tools::run_tool(opts);
    } catch (const std::exception& ex) {
        LOG_ERROR("factor_leverage_tool 运行失败: {}", ex.what());
        return 1;
    } catch (...) {
        LOG_ERROR("factor_leverage_tool 运行失败: 未知异常");
        return 1;
    }
}
