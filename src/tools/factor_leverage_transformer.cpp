#include "tools/factor_leverage_transformer.h"
#include "tools/kline_csv_loader.h"

#include "bridge/ingress.h"
#include "core/databus.h"
#include "factors/kline/low_freq_return_factor.h"
#include "factors/kline/ar1_return_factor.h"
#include "factors/kline/high_volume_remaining_factor.h"
#include "factors/kline/volume_ar_forecast_factor.h"
#include "factors/kline/volume_price_structure_factor.h"
#include "utils/fs_compat.h"
#include "utils/log.h"
#include "factor_tool_config.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <unordered_map>
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
    std::string factor_name;
    std::size_t lookback_days = 0;
    std::size_t leverage_window = SlidingGaussianLeverage::kDefaultWindow;
    std::size_t topic_capacity = 64'000;
};

namespace {

const FactorToolConfig& kToolConfig = GetFactorToolConfig();

struct FactorBinding {
    std::string key;
    std::string description;
    std::string input_topic;
    std::function<void(std::size_t)> register_topics;
    std::function<std::shared_ptr<BaseFactor>(const std::vector<std::string>&)> create_factor;
};

template <typename FactorT>
std::shared_ptr<BaseFactor> MakeBarOnlyFactor(const std::vector<std::string>& codes) {
    struct Wrapper final : public FactorT {
        using FactorT::FactorT;
        void on_quote(const QuoteDepth&) override {}
        void on_tick(const CombinedTick&) override {}
    };
    return std::make_shared<Wrapper>(codes);
}

const std::vector<FactorBinding>& GetFactorBindings() {
    static const std::vector<FactorBinding> bindings = {
        {
            "low_freq_return",
            "F7 低频谱 × 均值",
            "kline/ret_lowfreq_mu10",
            [](std::size_t cap) { LowFreqReturnFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return std::make_shared<LowFreqReturnFactor>(codes);
            }
        },
        {
            "ar1_return",
            "AR(1) 收益自回归系数",
            "kline/ar1_return_coeff",
            [](std::size_t cap) { Ar1ReturnFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<Ar1ReturnFactor>(codes);
            }
        },
        {
            "high_volume_remaining",
            "高量状态剩余时间",
            "kline/vol_high_remaining",
            [](std::size_t cap) { HighVolumeRemainingFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<HighVolumeRemainingFactor>(codes);
            }
        },
        {
            "volume_ar_forecast",
            "成交量 AR 预测比值",
            "kline/vol_ar1_pred_ratio",
            [](std::size_t cap) { VolumeArForecastFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<VolumeArForecastFactor>(codes);
            }
        },
        {
            "volume_price_structure_corr",
            "收益-对数成交量相关",
            "kline/ret_logvol_corr",
            [](std::size_t cap) { VolumePriceStructureFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<VolumePriceStructureFactor>(codes);
            }
        },
        {
            "volume_price_structure_pca1",
            "量价第一主成分收益载荷",
            "kline/pv_pca1_ret_loading",
            [](std::size_t cap) { VolumePriceStructureFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<VolumePriceStructureFactor>(codes);
            }
        },
    };
    return bindings;
}

const FactorBinding* ResolveFactorBinding(const std::string& key) {
    const auto& bindings = GetFactorBindings();
    for (const auto& binding : bindings) {
        if (binding.key == key) {
            return &binding;
        }
    }
    return nullptr;
}

void PrintFactorBindingList() {
    std::cerr << "可用因子 (--factor-name):" << std::endl;
    for (const auto& binding : GetFactorBindings()) {
        std::cerr << "  - " << binding.key << " : " << binding.description << std::endl;
    }
}

// 采样点结构：方便统一持久化 code/时间/数值
struct SamplePoint {
    std::string code;
    int64_t timestamp;
    double value;
};

struct FactorReturnSample {
    std::string code;
    int64_t timestamp;
    double factor_value;
    double return_value;
};

using FactorReturnTableMap = std::unordered_map<std::string, std::vector<FactorReturnSample>>;

struct CombinedSampleRow {
    std::string code;
    int64_t timestamp = 0;
    double factor_value = 0.0;
    bool has_factor = false;
    double return_value = 0.0;
    bool has_return = false;
    double leverage_value = 0.0;
    bool has_leverage = false;
    bool suppressed = false;
};

struct BaselineStats {
    double mean_return = 0.0;
    double std_return = 0.0;
    double sharpe_annual = 0.0;
    std::size_t sample_size = 0;
    bool sharpe_valid = false;
};

struct ThresholdResult {
    std::string code;
    double theta = 0.0;
    double theta_raw = 0.0;
    double mean_factor = 0.0;
    double std_factor = 1.0;
    double c = 0.0;
    std::size_t n_trades = 0;
    double mean_return = 0.0;
    double std_return = 0.0;
    double sharpe_annual = 0.0;
    BaselineStats baseline;
};

// 汇总器：订阅 DataBus 后，把原始因子存到向量
struct Collector {
    using TableMap = std::unordered_map<std::string, std::vector<SamplePoint>>;

    void add_factor(const std::string& table,
                    const std::string& code,
                    int64_t ts,
                    double value) {
        std::lock_guard<std::mutex> lk(mtx);
        factor_points[table].push_back({code, ts, value});
    }

    TableMap snapshot_factor() const {
        std::lock_guard<std::mutex> lk(mtx);
        return factor_points;
    }

private:
    mutable std::mutex mtx;
    TableMap factor_points;
};

using ReturnSeries = std::map<int64_t, double>;
using ReturnMap = std::unordered_map<std::string, ReturnSeries>;

ReturnMap build_return_map(const std::vector<KlineCsvSeries>& series_list) {
    ReturnMap ret_map;
    for (const auto& series : series_list) {
        const auto& code = series.code;
        const auto& bars = series.bars;
        if (bars.size() < 2) continue;
        for (std::size_t i = 0; i + 1 < bars.size(); ++i) {
            const auto& b0 = bars[i];
            const auto& b1 = bars[i + 1];
            if (b0.instrument_id != b1.instrument_id) continue;
            double c0 = b0.close;
            double c1 = b1.close;
            if (!std::isfinite(c0) || !std::isfinite(c1) || c0 == 0.0) continue;
            double r = c1 / c0 - 1.0;
            ret_map[code][b0.data_time_ms] = r;
        }
    }
    return ret_map;
}

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
    opts.leverage_window = kToolConfig.default_leverage_window;
    opts.topic_capacity = kToolConfig.default_topic_capacity;
    opts.lookback_days = kToolConfig.default_lookback_days;
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
        } else if (arg == "--factor-name") {
            if (auto v = next_value()) opts.factor_name = *v;
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
        } else if (arg == "--lookback-days") {
            if (auto v = next_value()) {
                opts.lookback_days = std::stoull(*v);
            }
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "用法: factor_leverage_tool [--data-dir DIR] [--output-dir DIR]\n"
                         "                    [--factor-name NAME] [--codes CODE1,CODE2]\n"
                         "                    [--start-date YYYY-MM-DD] [--end-date YYYY-MM-DD]\n"
                         "                    [--window N] [--lookback-days N]\n";
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

std::string sanitize_filename(const std::string& raw) {
    std::string clean;
    clean.reserve(raw.size());
    for (char ch : raw) {
        if (std::isalnum(static_cast<unsigned char>(ch)) ||
            ch == '-' || ch == '_') {
            clean.push_back(ch);
        } else {
            clean.push_back('_');
        }
    }
    if (clean.empty()) clean = "table";
    return clean;
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

std::optional<double> find_return(const ReturnMap& ret_map,
                                  const std::string& code,
                                  int64_t timestamp) {
    auto it_code = ret_map.find(code);
    if (it_code == ret_map.end()) return std::nullopt;
    auto it_ret = it_code->second.find(timestamp);
    if (it_ret == it_code->second.end()) return std::nullopt;
    return it_ret->second;
}

std::string make_sample_key(const std::string& code, int64_t timestamp) {
    return code + "|" + std::to_string(timestamp);
}

void ensure_directory(const fs::path& dir) {
    if (!fs::exists(dir)) {
        fs::create_directories(dir);
    }
}

void write_relation_csv(const fs::path& path,
                        const std::vector<FactorReturnSample>& points) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw std::runtime_error("无法写入文件: " + path.string());
    }
    ofs << "code,timestamp,factor_value,return_value\n";
    auto sorted = points;
    std::sort(sorted.begin(), sorted.end(),
              [](const FactorReturnSample& a, const FactorReturnSample& b) {
                  if (a.timestamp == b.timestamp) {
                      if (a.code == b.code) {
                          return a.factor_value < b.factor_value;
                      }
                      return a.code < b.code;
                  }
                  return a.timestamp < b.timestamp;
              });
    for (const auto& p : sorted) {
        ofs << p.code << ','
            << format_time(p.timestamp) << ','
            << std::setprecision(15) << p.factor_value << ','
            << std::setprecision(15) << p.return_value << '\n';
    }
}

// 生成临时 Python 脚本并调用 matplotlib 输出直方图
bool run_plot_script(const fs::path& script_path,
                     const fs::path& csv_path,
                     const fs::path& png_path,
                     const std::string& title,
                     const std::string& column_name,
                     const std::string& subtitle = std::string(),
                     const std::string& status_column = std::string()) {
    std::ofstream script(script_path);
    script << R"(import csv, sys, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def read_values(csv_path, value_col, status_col):
    active = []
    suppressed = []
    with open(csv_path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames or []
        has_status = status_col and status_col in headers
        if value_col not in headers:
            return active, suppressed
        for row in reader:
            try:
                v = float(row[value_col])
                if not math.isfinite(v):
                    continue
                if has_status:
                    status = row.get(status_col, "").strip().lower()
                    if status == "suppressed":
                        suppressed.append(v)
                    else:
                        active.append(v)
                else:
                    active.append(v)
            except Exception:
                pass
    return active, suppressed

def build_bins(values, bin_count):
    vmin = min(values)
    vmax = max(values)
    if math.isclose(vmin, vmax, rel_tol=1e-9, abs_tol=1e-12):
        span = abs(vmin) if abs(vmin) > 1e-6 else 1.0
        vmin -= span * 0.5
        vmax += span * 0.5
    return np.linspace(vmin, vmax, bin_count + 1)

def main():
    if len(sys.argv) < 5:
        print("Usage: python plot_hist.py data.csv output.png title column_name [subtitle] [status_column]")
        return 1
    csv_path, png_path, title, column_name = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    subtitle = sys.argv[5] if len(sys.argv) >= 6 else ""
    status_col = sys.argv[6] if len(sys.argv) >= 7 else ""
    active, suppressed = read_values(csv_path, column_name, status_col)
    combined = active + suppressed
    if not combined:
        print(f"[WARN] no values found in {csv_path}")
        return 0
    plt.figure(figsize=(8, 4.5))
    bins = 50
    bin_edges = build_bins(combined, bins)
    has_any = False
    if suppressed:
        plt.hist(suppressed, bins=bin_edges, color="#b0b0b0", alpha=0.95, edgecolor="white", label="|z| ≤ θ")
        has_any = True
    if active:
        plt.hist(active, bins=bin_edges, color="#1f77b4", alpha=0.85, edgecolor="white", label="|z| > θ")
        has_any = True
    if suppressed and active:
        plt.legend()
    if not has_any:
        print(f"[WARN] no values found in {csv_path}")
        return 0
    plt.title(title)
    plt.xlabel("value")
    plt.ylabel("frequency")
    plt.grid(alpha=0.3)
    if subtitle:
        plt.figtext(0.5, 0.02, subtitle, ha="center", fontsize=9)
        plt.tight_layout(rect=[0, 0.05, 1, 1])
    else:
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
        << title << "\" \"" << column_name << "\"";
    if (!subtitle.empty()) {
        std::string safe_sub = subtitle;
        std::replace(safe_sub.begin(), safe_sub.end(), '"', '\'');
        cmd << " \"" << safe_sub << "\"";
    } else {
        cmd << " \"\"";
    }
    if (!status_column.empty()) {
        cmd << " \"" << status_column << "\"";
    }
    int rc = std::system(cmd.str().c_str());
    return rc == 0;
}

bool run_relation_plot_script(const fs::path& script_path,
                              const fs::path& csv_path,
                              const fs::path& png_path,
                              const std::string& title) {
    std::ofstream script(script_path);
    script << R"(import csv, sys, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def read_pairs(csv_path):
    xs, ys = [], []
    with open(csv_path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                fx = float(row["factor_value"])
                ry = float(row["return_value"])
                if math.isfinite(fx) and math.isfinite(ry):
                    xs.append(fx)
                    ys.append(ry)
            except Exception:
                pass
    return xs, ys

def main():
    if len(sys.argv) < 4:
        print("Usage: python plot_factor_return.py data.csv output.png title")
        return 1
    csv_path, png_path, title = sys.argv[1], sys.argv[2], sys.argv[3]
    xs, ys = read_pairs(csv_path)
    if not xs:
        print(f"[WARN] no pairs found in {csv_path}")
        return 0
    plt.figure(figsize=(8, 4.5))
    plt.scatter(xs, ys, s=10, alpha=0.6, edgecolors='none', color='#ff7f0e')
    plt.title(title)
    plt.xlabel("factor value")
    plt.ylabel("next return")
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

void apply_lookback_limit(std::vector<KlineCsvSeries>& series_list, std::size_t lookback_days) {
    if (lookback_days == 0) return;
    for (auto& series : series_list) {
        auto& bars = series.bars;
        std::sort(bars.begin(), bars.end(),
                  [](const factorlib::Bar& a, const factorlib::Bar& b) {
                      if (a.data_time_ms == b.data_time_ms) {
                          return a.instrument_id < b.instrument_id;
                      }
                      return a.data_time_ms < b.data_time_ms;
                  });
        if (bars.size() > lookback_days) {
            auto erase_count = bars.size() - lookback_days;
            bars.erase(bars.begin(), bars.begin() + static_cast<std::ptrdiff_t>(erase_count));
        }
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

std::unordered_map<std::string, std::string> build_code_table_map(
    const std::vector<KlineCsvSeries>& series_list) {
    std::unordered_map<std::string, std::string> mapping;
    for (const auto& series : series_list) {
        mapping[series.code] = series.table_name;
    }
    return mapping;
}

std::vector<SamplePoint> flatten_tables(const Collector::TableMap& tables) {
    std::size_t total = 0;
    for (const auto& kv : tables) {
        total += kv.second.size();
    }
    std::vector<SamplePoint> all;
    all.reserve(total);
    for (const auto& kv : tables) {
        all.insert(all.end(), kv.second.begin(), kv.second.end());
    }
    return all;
}

std::vector<FactorReturnSample> flatten_relation_tables(const FactorReturnTableMap& tables) {
    std::size_t total = 0;
    for (const auto& kv : tables) {
        total += kv.second.size();
    }
    std::vector<FactorReturnSample> all;
    all.reserve(total);
    for (const auto& kv : tables) {
        all.insert(all.end(), kv.second.begin(), kv.second.end());
    }
    return all;
}

std::unordered_map<std::string, std::vector<FactorReturnSample>>
group_relation_by_code(const std::vector<FactorReturnSample>& samples) {
    std::unordered_map<std::string, std::vector<FactorReturnSample>> mapping;
    for (const auto& sample : samples) {
        mapping[sample.code].push_back(sample);
    }
    return mapping;
}

std::string format_threshold_value(double value) {
    std::ostringstream oss;
    if (std::abs(value) >= 1e-3) {
        oss << std::fixed << std::setprecision(4) << value;
    } else {
        oss << std::scientific << std::setprecision(2) << value;
    }
    return oss.str();
}

std::string build_threshold_subtitle(
    const std::vector<std::string>& codes,
    const std::unordered_map<std::string, ThresholdResult>& thresholds) {
    std::ostringstream oss;
    bool first = true;
    for (const auto& code : codes) {
        auto it = thresholds.find(code);
        if (it == thresholds.end()) continue;
        const auto& th = it->second;
        double leverage_thr = (th.theta > 1e-12) ? 1.0 : 0.0;
        if (!first) {
            oss << " | ";
        }
        first = false;
        oss << code << " θ_factor="
            << format_threshold_value(th.theta_raw)
            << " θ_leverage≈"
            << format_threshold_value(leverage_thr);
    }
    return oss.str();
}

std::vector<std::string> collect_codes_from_combined(const std::vector<CombinedSampleRow>& rows) {
    std::vector<std::string> codes;
    codes.reserve(rows.size());
    for (const auto& row : rows) {
        codes.push_back(row.code);
    }
    std::sort(codes.begin(), codes.end());
    codes.erase(std::unique(codes.begin(), codes.end()), codes.end());
    return codes;
}

std::vector<CombinedSampleRow> build_combined_rows_for_table(
    const std::string& table,
    const Collector::TableMap& factor_tables,
    const Collector::TableMap& leverage_tables,
    const ReturnMap& ret_map,
    const std::unordered_map<std::string, bool>& suppressed_flags) {
    std::unordered_map<std::string, CombinedSampleRow> rows;
    auto factor_it = factor_tables.find(table);
    if (factor_it == factor_tables.end()) {
        return {};
    }
    for (const auto& sp : factor_it->second) {
        auto key = make_sample_key(sp.code, sp.timestamp);
        auto [it, inserted] = rows.try_emplace(key);
        auto& row = it->second;
        if (inserted) {
            row.code = sp.code;
            row.timestamp = sp.timestamp;
        }
        row.factor_value = sp.value;
        row.has_factor = true;
        if (auto ret = find_return(ret_map, sp.code, sp.timestamp)) {
            row.return_value = *ret;
            row.has_return = true;
        }
    }

    auto leverage_it = leverage_tables.find(table);
    if (leverage_it != leverage_tables.end()) {
        for (const auto& sp : leverage_it->second) {
            auto key = make_sample_key(sp.code, sp.timestamp);
            auto [it, inserted] = rows.try_emplace(key);
            auto& row = it->second;
            if (inserted) {
                row.code = sp.code;
                row.timestamp = sp.timestamp;
            }
            row.leverage_value = sp.value;
            row.has_leverage = true;
            auto flag_it = suppressed_flags.find(key);
            if (flag_it != suppressed_flags.end()) {
                row.suppressed = flag_it->second;
            }
        }
    }

    std::vector<CombinedSampleRow> combined;
    combined.reserve(rows.size());
    for (auto& [_, row] : rows) {
        (void)_;
        combined.push_back(std::move(row));
    }
    std::sort(combined.begin(), combined.end(),
              [](const CombinedSampleRow& a, const CombinedSampleRow& b) {
                  if (a.timestamp == b.timestamp) {
                      return a.code < b.code;
                  }
                  return a.timestamp < b.timestamp;
              });
    return combined;
}

void write_combined_csv(const fs::path& path, const std::vector<CombinedSampleRow>& rows) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw std::runtime_error("无法写入文件: " + path.string());
    }
    ofs << "code,timestamp,factor_value,return_value,leverage_value,status\n";
    for (const auto& row : rows) {
        ofs << row.code << ','
            << format_time(row.timestamp) << ',';
        if (row.has_factor) {
            ofs << std::setprecision(15) << row.factor_value;
        }
        ofs << ',';
        if (row.has_return) {
            ofs << std::setprecision(15) << row.return_value;
        }
        ofs << ',';
        if (row.has_leverage) {
            ofs << std::setprecision(15) << row.leverage_value;
        }
        ofs << ','
            << (row.suppressed ? "suppressed" : "active")
            << '\n';
    }
}

BaselineStats compute_baseline_stats(const std::vector<FactorReturnSample>& samples) {
    BaselineStats stats;
    double sum = 0.0;
    double sum_sq = 0.0;
    for (const auto& sample : samples) {
        if (!std::isfinite(sample.return_value)) continue;
        sum += sample.return_value;
        sum_sq += sample.return_value * sample.return_value;
        ++stats.sample_size;
    }
    if (stats.sample_size > 0) {
        stats.mean_return = sum / static_cast<double>(stats.sample_size);
        double var = sum_sq / static_cast<double>(stats.sample_size) -
                     stats.mean_return * stats.mean_return;
        if (var > 1e-18) {
            stats.std_return = std::sqrt(var);
            double sharpe_daily = stats.mean_return / stats.std_return;
            stats.sharpe_annual = sharpe_daily * std::sqrt(252.0);
            stats.sharpe_valid = true;
        } else {
            stats.std_return = 0.0;
            stats.sharpe_annual = 0.0;
            stats.sharpe_valid = false;
        }
    }
    return stats;
}

std::optional<ThresholdResult> search_threshold_for_code(
    const std::string& code,
    const std::vector<FactorReturnSample>& samples) {
    const auto& cfg = kToolConfig;
    std::size_t count = 0;
    double sum = 0.0;
    double sum_sq = 0.0;
    for (const auto& sample : samples) {
        if (!std::isfinite(sample.factor_value)) continue;
        sum += sample.factor_value;
        sum_sq += sample.factor_value * sample.factor_value;
        ++count;
    }
    if (count < cfg.min_trade_days) {
        std::cout << "[THETA] code=" << code
                  << " 样本数不足，无法标准化 n=" << count << std::endl;
        return std::nullopt;
    }
    double mean_factor = sum / static_cast<double>(count);
    double var_factor = sum_sq / static_cast<double>(count) - mean_factor * mean_factor;
    if (var_factor <= 1e-18) {
        std::cout << "[THETA] code=" << code
                  << " 因子方差接近 0，无法标准化" << std::endl;
        return std::nullopt;
    }
    double std_factor = std::sqrt(var_factor);

    BaselineStats baseline = compute_baseline_stats(samples);
    if (baseline.sample_size == 0) {
        std::cout << "[THETA] code=" << code
                  << " 缺少基准收益数据" << std::endl;
        return std::nullopt;
    }

    ThresholdResult best;
    bool has_best = false;
    bool stop_due_to_trades = false;
    for (double theta = cfg.theta_min; theta <= cfg.theta_max + 1e-12; theta += cfg.theta_step) {
        double sum_b2 = 0.0;
        double max_abs_bt = 0.0;
        std::size_t n_open = 0;
        for (const auto& sample : samples) {
            double b = sample.factor_value;
            if (!std::isfinite(b)) continue;
            double z = (b - mean_factor) / std_factor;
            if (!std::isfinite(z)) continue;
            double abs_z = std::abs(z);
            double bt = (abs_z <= theta) ? 0.0 : z;
            if (bt == 0.0) continue;
            sum_b2 += bt * bt;
            max_abs_bt = std::max(max_abs_bt, std::abs(bt));
            ++n_open;
        }
        if (sum_b2 <= 0.0) {
            std::cout << "[THETA][skip] code=" << code
                      << " theta=" << theta << " 无有效样本或Σb^2=0" << std::endl;
            continue;
        }
        if (n_open < cfg.min_trade_days) {
            std::cout << "[THETA][skip] code=" << code
                      << " theta=" << theta << " 交易天数不足 n=" << n_open << std::endl;
            stop_due_to_trades = true;
            break;
        }
        double c_risk = std::sqrt(static_cast<double>(n_open) / sum_b2);
        double c_max = max_abs_bt > 0.0 ? (cfg.max_leverage / max_abs_bt) : c_risk;
        double c = std::min(c_risk, c_max);

        double sum_R = 0.0;
        double sum_R2 = 0.0;
        std::size_t n_trades = 0;
        for (const auto& sample : samples) {
            double b = sample.factor_value;
            double r = sample.return_value;
            if (!std::isfinite(b) || !std::isfinite(r)) continue;
            double z = (b - mean_factor) / std_factor;
            if (!std::isfinite(z)) continue;
            double abs_z = std::abs(z);
            double bt = (abs_z <= theta) ? 0.0 : z;
            if (bt == 0.0) continue;
            double L = c * bt;
            double Rt = L * r;
            sum_R += Rt;
            sum_R2 += Rt * Rt;
            ++n_trades;
        }
        if (n_trades < cfg.min_trade_days) {
            std::cout << "[THETA][skip] code=" << code
                      << " theta=" << theta << " n_trades=" << n_trades
                      << " < " << cfg.min_trade_days << std::endl;
            stop_due_to_trades = true;
            break;
        }
        double mean_R = sum_R / static_cast<double>(n_trades);
        double var_R = sum_R2 / static_cast<double>(n_trades) - mean_R * mean_R;
        if (var_R <= 1e-18) {
            std::cout << "[THETA][skip] code=" << code
                      << " theta=" << theta << " 方差过小 var=" << var_R << std::endl;
            continue;
        }
        double std_R = std::sqrt(var_R);
        if (std_R <= 0.0) {
            std::cout << "[THETA][skip] code=" << code
                      << " theta=" << theta << " std=0" << std::endl;
            continue;
        }
        double sharpe_daily = mean_R / std_R;
        double sharpe_annual = sharpe_daily * std::sqrt(252.0);
        if (mean_R <= baseline.mean_return + 1e-12) {
            std::cout << "[THETA][skip] code=" << code
                      << " theta=" << theta << " mean<=bench mean_R=" << mean_R
                      << " mean_bench=" << baseline.mean_return << std::endl;
            continue;
        }
        if (!has_best || theta < best.theta) {
            has_best = true;
            best.code = code;
            best.theta = theta;
            best.theta_raw = theta * std_factor;
            best.mean_factor = mean_factor;
            best.std_factor = std_factor;
            best.c = c;
            best.n_trades = n_trades;
            best.mean_return = mean_R;
            best.std_return = std_R;
            best.sharpe_annual = sharpe_annual;
            best.baseline = baseline;
        }
    }
    if (stop_due_to_trades && !has_best) {
        return std::nullopt;
    }
    if (has_best) {
        return best;
    }
    return std::nullopt;
}

void write_threshold_results_csv(const fs::path& path,
                                 const std::vector<ThresholdResult>& results) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw std::runtime_error("无法写入文件: " + path.string());
    }
    ofs << "code,theta_z,theta_raw,c,n_trades,mean_return,std_return,sharpe_annual,"
           "mean_factor,std_factor,baseline_mean,baseline_std,baseline_sharpe,baseline_samples\n";
    for (const auto& res : results) {
        ofs << res.code << ','
            << std::setprecision(15) << res.theta << ','
            << std::setprecision(15) << res.theta_raw << ','
            << std::setprecision(15) << res.c << ','
            << res.n_trades << ','
            << std::setprecision(15) << res.mean_return << ','
            << std::setprecision(15) << res.std_return << ','
            << std::setprecision(15) << res.sharpe_annual << ','
            << std::setprecision(15) << res.mean_factor << ','
            << std::setprecision(15) << res.std_factor << ','
            << std::setprecision(15) << res.baseline.mean_return << ','
            << std::setprecision(15) << res.baseline.std_return << ','
            << std::setprecision(15) << res.baseline.sharpe_annual << ','
            << res.baseline.sample_size << '\n';
    }
}

} // namespace

CliOptions parse_cli_options(int argc, char** argv) {
    return parse_args_internal(argc, argv);
}

int run_factor_binding(const FactorBinding& binding, const CliOptions& opts) {
    KlineCsvLoader loader(opts.data_dir);
    auto series_list = loader.load(opts.codes, opts.start_ms, opts.end_ms);
    if (series_list.empty()) {
        std::cerr << "[ERROR] 在目录 " << opts.data_dir << " 中找不到匹配的 CSV。" << std::endl;
        return 1;
    }
    if (opts.lookback_days > 0) {
        apply_lookback_limit(series_list, opts.lookback_days);
        std::cout << "[INFO] 仅使用最近 " << opts.lookback_days << " 条 Bar 数据" << std::endl;
    }

    ReturnMap ret_map = build_return_map(series_list);
    auto codes = gather_codes(series_list);
    auto code_table_map = build_code_table_map(series_list);
    std::cout << "[INFO] 处理代码数量: " << codes.size() << std::endl;

    DataBus::instance().reset();
    binding.register_topics(opts.topic_capacity);
    auto factor = binding.create_factor(codes);
    if (!factor) {
        std::cerr << "[ERROR] 因子 " << binding.key << " 创建失败。" << std::endl;
        return 1;
    }
    bridge::set_factors({factor});

    FactorLeverageSpec spec;
    spec.input_topic = binding.input_topic;
    spec.output_topic = binding.input_topic + "_leverage";
    spec.window = opts.leverage_window;
    FactorLeverageTransformer transformer({spec}, codes, opts.topic_capacity);
    transformer.start();

    Collector collector;
    for (const auto& code : codes) {
        const auto it = code_table_map.find(code);
        const std::string table_name = (it != code_table_map.end()) ? it->second : code;
        // 订阅原始因子，用于输出“未高斯化”的分布图
        DataBus::instance().subscribe<double>(
            spec.input_topic,
            code,
            [&collector, table_name](const std::string& cb_code, int64_t ts, const double& value) {
                if (std::isfinite(value)) {
                    collector.add_factor(table_name, cb_code, ts, value);
                }
            });
    }

    auto bars = collect_bars(series_list);
    std::cout << "[INFO] 推送 Bar 数量: " << bars.size() << std::endl;
    drive_factors(bars);
    transformer.stop();

    auto factor_tables = collector.snapshot_factor();

    FactorReturnTableMap relation_tables;
    for (const auto& [table, samples] : factor_tables) {
        std::vector<FactorReturnSample> rel_samples;
        rel_samples.reserve(samples.size());
        for (const auto& sp : samples) {
            auto it_code = ret_map.find(sp.code);
            if (it_code == ret_map.end()) continue;
            auto it_ret = it_code->second.find(sp.timestamp);
            if (it_ret == it_code->second.end()) continue;
            double r = it_ret->second;
            if (!std::isfinite(sp.value) || !std::isfinite(r)) continue;
            rel_samples.push_back(FactorReturnSample{
                sp.code,
                sp.timestamp,
                sp.value,
                r
            });
        }
        if (!rel_samples.empty()) {
            relation_tables.emplace(table, std::move(rel_samples));
        }
    }
    auto relation_all = flatten_relation_tables(relation_tables);
    auto code_relation_map = group_relation_by_code(relation_all);

    std::unordered_map<std::string, ThresholdResult> code_thresholds;
    std::vector<ThresholdResult> threshold_results;
    std::cout << "[INFO] ===== 阈值网格搜索 =====" << std::endl;
    for (const auto& [code, samples] : code_relation_map) {
        auto result = search_threshold_for_code(code, samples);
        if (result) {
            threshold_results.push_back(*result);
            code_thresholds[code] = *result;
            std::cout << "[THETA] code=" << code
                      << " theta_z=" << result->theta
                      << " theta_raw≈" << result->theta_raw
                      << " c=" << result->c
                      << " trades=" << result->n_trades
                      << " mean=" << result->mean_return
                      << " mean_bench=" << result->baseline.mean_return
                      << " std=" << result->std_return
                      << " sharpe_annual=" << result->sharpe_annual
                      << " baseline_sharpe=" << result->baseline.sharpe_annual
                      << std::endl;
        } else {
            std::cout << "[THETA] code=" << code << " 无满足约束的阈值" << std::endl;
        }
    }

    std::unordered_map<std::string, double> code_max_abs_z;
    for (const auto& [table, samples] : factor_tables) {
        (void)table;
        for (const auto& sp : samples) {
            auto th_it = code_thresholds.find(sp.code);
            if (th_it == code_thresholds.end() || th_it->second.std_factor <= 0.0) {
                continue;
            }
            const auto& th = th_it->second;
            double z = (sp.value - th.mean_factor) / th.std_factor;
            if (!std::isfinite(z)) {
                continue;
            }
            double abs_z = std::abs(z);
            if (abs_z <= th.theta + 1e-12) {
                continue;
            }
            auto& max_abs = code_max_abs_z[sp.code];
            if (abs_z > max_abs) {
                max_abs = abs_z;
            }
        }
    }

    Collector::TableMap leverage_tables;
    std::unordered_map<std::string, bool> suppressed_flags;
    const double max_allowed = std::max(1.0, kToolConfig.max_leverage);
    for (const auto& [table, samples] : factor_tables) {
        auto& lev_vec = leverage_tables[table];
        lev_vec.reserve(samples.size());
        for (const auto& sp : samples) {
            bool suppressed = true;
            double leverage_value = 0.0;
            auto th_it = code_thresholds.find(sp.code);
            if (th_it != code_thresholds.end() && th_it->second.std_factor > 0.0) {
                const auto& th = th_it->second;
                double z = (sp.value - th.mean_factor) / th.std_factor;
                if (std::isfinite(z) && std::abs(z) > th.theta + 1e-12) {
                    suppressed = false;
                    double max_z = th.theta;
                    auto max_it = code_max_abs_z.find(sp.code);
                    if (max_it != code_max_abs_z.end()) {
                        max_z = std::max(max_it->second, th.theta);
                    }
                    if (max_z <= th.theta + 1e-12) {
                        max_z = th.theta + 1.0;
                    }
                    double span = std::max(max_z - th.theta, 1e-12);
                    double frac = (std::abs(z) - th.theta) / span;
                    frac = std::clamp(frac, 0.0, 1.0);
                    double magnitude = 1.0 + frac * (max_allowed - 1.0);
                    magnitude = std::clamp(magnitude, 1.0, max_allowed);
                    leverage_value = std::copysign(magnitude, z);
                }
            }
            lev_vec.push_back(SamplePoint{sp.code, sp.timestamp, leverage_value});
            auto key = make_sample_key(sp.code, sp.timestamp);
            suppressed_flags[key] = suppressed;
        }
    }

    std::string factor_dir = binding.key;
    auto factor_dir_safe = sanitize_filename(factor_dir);
    if (factor_dir_safe.empty()) {
        factor_dir_safe = "factor";
    }
    fs::path output_root = fs::path(opts.output_dir);
    ensure_directory(output_root);
    fs::path output_dir = output_root / factor_dir_safe;
    ensure_directory(output_dir);
    if (!threshold_results.empty()) {
        fs::path threshold_csv = output_dir / "leverage_thresholds.csv";
        write_threshold_results_csv(threshold_csv, threshold_results);
    } else {
        std::cout << "[WARN] 阈值搜索未得到有效结果，未输出阈值文件。" << std::endl;
    }

#if FACTORLIB_ENABLE_FACTOR_LEVERAGE_PLOTS
    fs::path hist_script_path = output_root / "plot_hist.py";
    fs::path relation_script_path = output_root / "plot_factor_return.py";
    std::size_t combined_tables = 0;
    for (const auto& [table, _] : factor_tables) {
        auto combined_rows = build_combined_rows_for_table(
            table, factor_tables, leverage_tables, ret_map, suppressed_flags);
        if (combined_rows.empty()) {
            continue;
        }
        ++combined_tables;
        auto safe = sanitize_filename(table);
        fs::path csv_path = output_dir / ("factor_table_" + safe + ".csv");
        write_combined_csv(csv_path, combined_rows);
        auto codes = collect_codes_from_combined(combined_rows);
        bool has_threshold = std::any_of(
            codes.begin(), codes.end(),
            [&code_thresholds](const std::string& code) {
                return code_thresholds.find(code) != code_thresholds.end();
            });
        std::string subtitle;
        if (has_threshold) {
            subtitle = build_threshold_subtitle(codes, code_thresholds);
        }

        fs::path factor_png = output_dir / ("factor_distribution_" + safe + ".png");
        std::string factor_title = "factor_distribution (" + table + ")";
        if (!run_plot_script(hist_script_path,
                             csv_path,
                             factor_png,
                             factor_title,
                             "factor_value")) {
            std::cerr << "[WARN] 因子图绘制失败: " << factor_png << std::endl;
        }

        if (has_threshold) {
            fs::path leverage_png = output_dir / ("leverage_distribution_" + safe + ".png");
            std::string leverage_title = "leverage_distribution (" + table + ")";
            if (!run_plot_script(hist_script_path,
                                 csv_path,
                                 leverage_png,
                                 leverage_title,
                                 "leverage_value",
                                 subtitle,
                                 "status")) {
                std::cerr << "[WARN] 杠杆图绘制失败: " << leverage_png << std::endl;
            }
        } else {
            std::cout << "[INFO] 表 " << table
                      << " 没有可用阈值，跳过杠杆分布图。" << std::endl;
        }

        fs::path relation_png = output_dir / ("factor_return_relation_" + safe + ".png");
        std::string relation_title = "Factor vs Next Return (" + table + ")";
        if (!run_relation_plot_script(
                relation_script_path,
                csv_path,
                relation_png,
                relation_title)) {
            std::cerr << "[WARN] 因子-收益关系图绘制失败: " << relation_png << std::endl;
        }
    }
    std::cout << "[INFO] 按表输出数量: combined=" << combined_tables << std::endl;
#else
    std::cout << "[INFO] 已禁用按表导出。" << std::endl;
#endif

    return 0;
}

int run_tool(const CliOptions& opts) {
    std::vector<const FactorBinding*> targets;
    if (opts.factor_name.empty() || opts.factor_name == "all") {
        const auto& all = GetFactorBindings();
        for (const auto& binding : all) {
            targets.push_back(&binding);
        }
    } else {
        auto binding = ResolveFactorBinding(opts.factor_name);
        if (!binding) {
            std::cerr << "[ERROR] 未找到名为 \"" << opts.factor_name
                      << "\" 的因子绑定。" << std::endl;
            PrintFactorBindingList();
            return 1;
        }
        targets.push_back(binding);
    }

    int exit_code = 0;
    for (const auto* binding : targets) {
        std::cout << "[INFO] ===== 因子: " << binding->key << " =====" << std::endl;
        int rc = run_factor_binding(*binding, opts);
        if (rc != 0) {
            exit_code = rc;
        }
    }
    return exit_code;
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
