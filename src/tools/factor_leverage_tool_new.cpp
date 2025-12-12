#include "factor_leverage_optimizer.h"
#include "factor_leverage_tool_config.h"
#include "kline_csv_loader.h"

#include "core/databus.h"
#include "core/types.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

using namespace factorlib::tools;

namespace {

std::optional<std::string> get_arg(int argc, char** argv, const std::string& key) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (argv[i] == key) return std::string(argv[i + 1]);
    }
    return std::nullopt;
}

bool has_flag(int argc, char** argv, const std::string& key) {
    for (int i = 1; i < argc; ++i) {
        if (argv[i] == key) return true;
    }
    return false;
}

std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == ',') {
            if (!cur.empty()) out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

std::optional<int64_t> parse_date_utc_ms(const std::string& ymd) {
    if (ymd.size() < 10) return std::nullopt;
    int y = std::stoi(ymd.substr(0, 4));
    int m = std::stoi(ymd.substr(5, 2));
    int d = std::stoi(ymd.substr(8, 2));
    std::tm tm{};
    tm.tm_year = y - 1900;
    tm.tm_mon = m - 1;
    tm.tm_mday = d;
#if defined(_WIN32)
    std::time_t sec = _mkgmtime(&tm);
#else
    std::time_t sec = timegm(&tm);
#endif
    return static_cast<int64_t>(sec) * 1000;
}

std::string format_date_utc(int64_t ms) {
    std::time_t sec = static_cast<std::time_t>(ms / 1000);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &sec);
#else
    gmtime_r(&sec, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d");
    return oss.str();
}

// FactorBinding/默认目录 已移到 factor_leverage_tool_config.*

struct RetPoint { int64_t ts_ms; double r; };

std::vector<RetPoint> compute_next_returns(const std::vector<factorlib::Bar>& bars) {
    std::vector<RetPoint> out;
    if (bars.size() < 2) return out;
    out.reserve(bars.size() - 1);
    for (std::size_t i = 0; i + 1 < bars.size(); ++i) {
        const auto& b0 = bars[i];
        const auto& b1 = bars[i + 1];
        if (b0.close <= 0.0) continue;
        double r = b1.close / b0.close - 1.0;
        if (!std::isfinite(r)) continue;
        out.push_back({b0.data_time_ms, r});
    }
    return out;
}

void write_csv(const fs::path& path,
               const std::vector<factorlib::tools::LeveragePoint>& pts,
               const factorlib::tools::ThresholdSearchResult& best,
               const std::string& code,
               const std::string& factor_key) {
    std::ofstream out(path);
    out << "date,code,factor,factor_raw,z,mode,polarity,theta,z_cap,c_scale,b_raw,leverage,next_return,equity,drawdown,active,profile_kind,symmetry_score,unique_ratio,max_freq_ratio\n";
    auto kind_to_str = [](factorlib::tools::FactorDistributionKind k) {
        switch (k) {
            case factorlib::tools::FactorDistributionKind::SymmetricContinuous: return "symmetric";
            case factorlib::tools::FactorDistributionKind::Asymmetric: return "asymmetric";
            default: return "discrete";
        }
    };
    auto mode_to_str = [](factorlib::tools::TradeMode m) {
        switch (m) {
            case factorlib::tools::TradeMode::BothSides: return "both";
            case factorlib::tools::TradeMode::PositiveOnly: return "pos_only";
            default: return "neg_only";
        }
    };

    for (const auto& p : pts) {
        out << format_date_utc(p.ts_ms) << ","
            << code << ","
            << factor_key << ","
            << p.x_raw << ","
            << p.z << ","
            << mode_to_str(best.mode) << ","
            << best.polarity << ","
            << best.theta << ","
            << best.z_cap << ","
            << best.c_scale << ","
            << p.b_raw << ","
            << p.leverage << ","
            << p.ret << ","
            << p.equity << ","
            << p.drawdown << ","
            << (p.active ? 1 : 0) << ","
            << kind_to_str(best.profile.kind) << ","
            << best.profile.symmetry_score << ","
            << best.profile.unique_ratio << ","
            << best.profile.max_freq_ratio
            << "\n";
    }
}

void append_summary(const fs::path& path,
                    const factorlib::tools::ThresholdSearchResult& best,
                    const std::string& code,
                    const std::string& factor_key) {
    const bool exists = fs::exists(path);
    std::ofstream out(path, std::ios::app);
    if (!exists) {
        out << "code,factor,score,baseline_score,theta,mode,polarity,c_scale,z_cap,final_equity,dd_rms,trade_days,D_sample_days,T_trade_days,profile_kind,symmetry_score,unique_ratio,max_freq_ratio,raw_threshold_low,raw_threshold_high\n\n";
    }
    auto kind_to_str = [](factorlib::tools::FactorDistributionKind k) {
        switch (k) {
            case factorlib::tools::FactorDistributionKind::SymmetricContinuous: return "symmetric";
            case factorlib::tools::FactorDistributionKind::Asymmetric: return "asymmetric";
            default: return "discrete";
        }
    };
    auto mode_to_str = [](factorlib::tools::TradeMode m) {
        switch (m) {
            case factorlib::tools::TradeMode::BothSides: return "both";
            case factorlib::tools::TradeMode::PositiveOnly: return "pos_only";
            default: return "neg_only";
        }
    };

    out << code << ","
        << factor_key << ","
        << best.score << ","
        << best.baseline_score << ","
        << best.theta << ","
        << mode_to_str(best.mode) << ","
        << best.polarity << ","
        << best.c_scale << ","
        << best.z_cap << ","
        << best.final_equity << ","
        << best.dd_rms << ","
        << best.trade_days << ","
        << best.D_sample_days << ","
        << best.T_trade_days << ","
        << kind_to_str(best.profile.kind) << ","
        << best.profile.symmetry_score << ","
        << best.profile.unique_ratio << ","
        << best.profile.max_freq_ratio << ","
        << best.theta_raw_low << ","
        << best.theta_raw_high
        << "\n";
}

bool run_triple_hist_plot(const fs::path& script_path,
                          const fs::path& csv_path,
                          const fs::path& png_path,
                          const std::string& title,
                          const std::string& factor_col,
                          const std::string& b_col,
                          const std::string& leverage_col,
                          const std::string& theta_text,
                          const std::string& score_text,
                          const std::string& baseline_text) {
    std::ofstream script(script_path);
    script << R"PLOT(import csv, sys, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def read_column(csv_path, col_name):
    values = []
    with open(csv_path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames or []
        if col_name not in headers:
            return values
        for row in reader:
            try:
                v = float(row[col_name])
                if math.isfinite(v):
                    values.append(v)
            except Exception:
                pass
    return values

def plot_hist(ax, values, title):
    if not values:
        ax.set_title(title + " (empty)")
        ax.grid(alpha=0.3)
        return
    v = np.array(values, dtype=float)
    ax.hist(v, bins=60)
    ax.set_title(title)
    ax.grid(alpha=0.3)

def main():
    # argv: data.csv out.png title factor_col b_col leverage_col theta_text
    if len(sys.argv) < 10:
        print("Usage: python plot_triple_hist.py data.csv output.png title factor_col b_col leverage_col theta_text score_text baseline_text")
        return 1
    csv_path, png_path, title, factor_col, b_col, leverage_col, theta_text, score_text, baseline_text = sys.argv[1:10]

    factor_vals = read_column(csv_path, factor_col)
    b_vals = read_column(csv_path, b_col)
    lev_vals = read_column(csv_path, leverage_col)

    if not factor_vals and not b_vals and not lev_vals:
        print(f"[WARN] no numeric values found in {csv_path}")
        return 0

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    plot_hist(axes[0], factor_vals, f"{factor_col} distribution")
    plot_hist(axes[1], b_vals, f"{b_col} distribution (b_t)")
    plot_hist(axes[2], lev_vals, f"{leverage_col} distribution (L_t)")

    fig.suptitle(title)
    subtitle_items = []
    if theta_text:
        subtitle_items.append(theta_text)
    if score_text:
        subtitle_items.append(score_text)
    if baseline_text:
        subtitle_items.append(baseline_text)
    if subtitle_items:
        fig.text(0.5, 0.02, " | ".join(subtitle_items), ha="center", fontsize=9)

    fig.tight_layout(rect=[0, 0.05, 1, 0.95])
    fig.savefig(png_path)
    return 0

if __name__ == "__main__":
    sys.exit(main())
 )PLOT";
    script.close();

    std::ostringstream safe_title;
    safe_title << title;
    std::string title_str = safe_title.str();
    std::replace(title_str.begin(), title_str.end(), '"', '\'');

    std::ostringstream cmd;
    cmd << "python \"" << script_path.string() << "\" \""
        << csv_path.string() << "\" \"" << png_path.string() << "\" \""
        << title_str << "\" \"" << factor_col << "\" \"" << b_col << "\" \""
        << leverage_col << "\" \"" << theta_text << "\" \"" << score_text << "\" \""
        << baseline_text << "\"";
    int rc = std::system(cmd.str().c_str());
    return rc == 0;
}

} // namespace

int main(int argc, char** argv) {
    if (has_flag(argc, argv, "--help")) {
        std::cout
            << "Usage:\n"
            << "  factor_leverage_tool_new \\\n"
            << "    [--data_dir DIR]                (default tests/data) \\\n"
            << "    [--factor name1,name2|all]      (default all built-ins) \\\n"
            << "    [--codes CODE1,CODE2,...]       (default load every CSV) \\\n"
            << "    [--start YYYY-MM-DD] [--end YYYY-MM-DD] \\\n"
            << "    [--out_dir DIR]                 (default output/factor_leverage) \\\n"
            << "    [--theta_min 0] [--theta_max 2.5] [--theta_step 0.05] \\\n"
            << "    [--D 250] [--max_leverage 2]\n";
        return 0;
    }

    std::string data_dir = get_arg(argc, argv, "--data_dir").value_or(DefaultDataDir());

    std::vector<std::string> requested_factors;
    if (auto factor_arg = get_arg(argc, argv, "--factor")) {
        requested_factors = split_csv(*factor_arg);
    }
    if (requested_factors.size() == 1 && requested_factors.front() == "all") {
        requested_factors.clear();
    }
    if (requested_factors.empty()) {
        for (const auto& b : GetFactorBindings()) requested_factors.push_back(b.key);
    }

    std::vector<std::string> codes;
    if (auto codes_csv = get_arg(argc, argv, "--codes")) {
        codes = split_csv(*codes_csv);
    }

    std::string out_dir_base =
        get_arg(argc, argv, "--out_dir").value_or(DefaultOutputDir());
    fs::create_directories(out_dir_base);

    factorlib::tools::LeverageSearchConfig cfg;
    if (auto v = get_arg(argc, argv, "--theta_min")) cfg.theta_min = std::stod(*v);
    if (auto v = get_arg(argc, argv, "--theta_max")) cfg.theta_max = std::stod(*v);
    if (auto v = get_arg(argc, argv, "--theta_step")) cfg.theta_step = std::stod(*v);
    bool D_overridden = false;
    if (auto v = get_arg(argc, argv, "--D")) { cfg.D = std::stoi(*v); D_overridden = true; }
    if (auto v = get_arg(argc, argv, "--max_leverage")) cfg.max_leverage = std::stod(*v);

    factorlib::tools::FactorLeverageOptimizer opt(cfg);
    factorlib::tools::KlineCsvLoader loader(data_dir);

    std::optional<int64_t> start_ms;
    std::optional<int64_t> end_ms;
    if (auto v = get_arg(argc, argv, "--start")) start_ms = parse_date_utc_ms(*v);
    if (auto v = get_arg(argc, argv, "--end")) end_ms = parse_date_utc_ms(*v);

    auto series_list = loader.load(codes, start_ms, end_ms);
    if (series_list.empty()) {
        std::cerr << "No series loaded. Check --data_dir / --codes.\n";
        return 2;
    }

    bool any_factor_run = false;

    for (const auto& factor_name : requested_factors) {
        auto binding_ptr = ResolveFactorBinding(factor_name);
        if (!binding_ptr) {
            std::cerr << "Unknown factor: " << factor_name << "\n";
            continue;
        }
        const auto& binding = *binding_ptr;
        any_factor_run = true;

        factorlib::DataBus::instance().reset();
        constexpr std::size_t cap = 8192;
        binding.register_topics(cap);

        fs::path factor_out_dir = fs::path(out_dir_base) / factor_name;
        fs::create_directories(factor_out_dir);
        fs::path summary_path = factor_out_dir / "best_thresholds.csv";
        if (fs::exists(summary_path)) {
            std::error_code ec;
            fs::remove(summary_path, ec);
        }
        fs::path plot_script = factor_out_dir / "plot_distribution.py";

        bool any_code_processed = false;

        for (const auto& series : series_list) {
            const std::string& code = series.code;
            auto bars = series.bars;
            // D 用于样本裁剪：若指定 --D，则仅使用最近 D 天；若 D > 样本长度则兜底为样本长度。
            const int desired_D_days =
                (D_overridden && cfg.D > 0) ? cfg.D : static_cast<int>(bars.size());
            const int effective_D_days =
                std::min(desired_D_days, static_cast<int>(bars.size()));
            if (D_overridden && cfg.D > 0 && bars.size() > static_cast<std::size_t>(effective_D_days)) {
                auto erase_count = bars.size() - static_cast<std::size_t>(effective_D_days);
                bars.erase(bars.begin(), bars.begin() + static_cast<std::ptrdiff_t>(erase_count));
            }
            if (bars.size() < 5) {
                std::cerr << "[" << factor_name << "] skip " << code << " (too few bars)\n";
                continue;
            }

            auto rets = compute_next_returns(bars);
            // D_sample_days 取裁剪后的原始样本天数，用于风险对齐与年均交易日约束。
            int D_sample_days = effective_D_days;
            if (rets.size() < 5) {
                std::cerr << "[" << factor_name << "] skip " << code << " (too few returns)\n";
                continue;
            }
            std::unordered_map<int64_t, double> ret_by_ts;
            ret_by_ts.reserve(rets.size() * 2);
            for (const auto& rp : rets) ret_by_ts[rp.ts_ms] = rp.r;

            std::mutex mtx;
            std::vector<std::pair<int64_t, double>> factor_pts;
            factor_pts.reserve(bars.size());

            factorlib::DataBus::instance().subscribe<double>(
                binding.input_topic,
                code,
                [&mtx, &factor_pts](const std::string&, int64_t ts, const double& v) {
                    std::lock_guard<std::mutex> lk(mtx);
                    factor_pts.emplace_back(ts, v);
                });

            auto factor = binding.create({code});
            if (!factor) {
                std::cerr << "[" << factor_name << "] failed to create factor for " << code << "\n";
                continue;
            }
            for (const auto& b : bars) {
                factor->on_bar(b);
                factor->force_flush(b.instrument_id);
            }

            std::vector<int64_t> ts;
            std::vector<double> x_raw;
            std::vector<double> next_ret;

            {
                std::lock_guard<std::mutex> lk(mtx);
                std::sort(factor_pts.begin(), factor_pts.end(),
                          [](const auto& a, const auto& b) { return a.first < b.first; });
            }

            {
                std::lock_guard<std::mutex> lk(mtx);
                for (const auto& [t, x] : factor_pts) {
                    auto it = ret_by_ts.find(t);
                    if (it == ret_by_ts.end()) continue;
                    if (!std::isfinite(x)) continue;
                    ts.push_back(t);
                    x_raw.push_back(x);
                    next_ret.push_back(it->second);
                }
            }

            if (ts.size() < 30) {
                std::cerr << "[" << factor_name << "] skip " << code << " (aligned samples < 30)\n";
                continue;
            }

            auto profile = opt.analyze_profile(x_raw);
            std::vector<double> centered;
            centered.reserve(x_raw.size());
            for (double v : x_raw) centered.push_back(v - profile.median);
            auto z = opt.rank_normalize_to_z(centered);

            auto best = opt.search_best_threshold(ts, x_raw, z, next_ret, D_sample_days);
            if (!best.ok) {
                std::cerr << "[" << factor_name << "] no valid threshold for " << code << "\n";
                std::vector<factorlib::tools::LeveragePoint> placeholder;
                placeholder.reserve(ts.size());
                for (std::size_t i = 0; i < ts.size(); ++i) {
                    factorlib::tools::LeveragePoint p;
                    p.ts_ms = ts[i];
                    p.x_raw = x_raw[i];
                    p.z = z[i];
                    p.ret = next_ret[i];
                    placeholder.push_back(p);
                }
                factorlib::tools::ThresholdSearchResult dummy;
                dummy.ok = false;
                dummy.theta = std::numeric_limits<double>::quiet_NaN();
                dummy.c_scale = 0.0;
                dummy.z_cap = std::numeric_limits<double>::quiet_NaN();
                fs::path csv_path = factor_out_dir / ("leverage_" + code + "_" + factor_name + ".csv");
                write_csv(csv_path, placeholder, dummy, code, factor_name);
                fs::path dist_png = factor_out_dir / ("distribution_" + code + "_" + factor_name + ".png");
                if (!run_triple_hist_plot(plot_script,
                                        csv_path,
                                        dist_png,
                                        factor_name + " | " + code,
                                        "factor_raw",
                                        "b_raw",
                                        "leverage",
                                        "NaN",
                                        "score=NaN",
                                        "baseline=NaN")) {
                    std::cerr << "[" << factor_name << "] 绘制分布图失败: " << dist_png << std::endl;
                }
                continue;
            }

            auto pts = opt.build_leverage_series(ts, x_raw, z, next_ret, D_sample_days, best);

            fs::path csv_path = factor_out_dir / ("leverage_" + code + "_" + factor_name + ".csv");
            write_csv(csv_path, pts, best, code, factor_name);
            append_summary(summary_path, best, code, factor_name);

            fs::path dist_png = factor_out_dir / ("distribution_" + code + "_" + factor_name + ".png");
            std::string plot_title = factor_name + " | " + code;
            std::ostringstream theta_stream;
            theta_stream << std::fixed << std::setprecision(4)
                         << "z=" << best.theta;
            if (!std::isnan(best.theta_raw_high)) {
                theta_stream << ", x_high=" << best.theta_raw_high;
            }
            if (!std::isnan(best.theta_raw_low)) {
                theta_stream << ", x_low=" << best.theta_raw_low;
            }
            std::string theta_label = theta_stream.str();
            std::ostringstream score_stream;
            score_stream << std::fixed << std::setprecision(4)
                         << "score=" << best.score;
            std::string score_label = score_stream.str();
            std::ostringstream baseline_stream;
            baseline_stream << std::fixed << std::setprecision(4)
                            << "baseline=" << best.baseline_score;
            std::string baseline_label = baseline_stream.str();
            if (!run_triple_hist_plot(plot_script,
                                    csv_path,
                                    dist_png,
                                    plot_title,
                                    "factor_raw",
                                    "b_raw",
                                    "leverage",
                                    theta_label,
                                    score_label,
                                    baseline_label)) {
                std::cerr << "[" << factor_name << "] 绘制分布图失败: " << dist_png << std::endl;
            }

            std::cout << "[" << factor_name << "] OK " << code
                      << " score=" << best.score
                      << " theta=" << best.theta
                      << " c=" << best.c_scale
                      << " mode=" << static_cast<int>(best.mode)
                      << " polarity=" << best.polarity
                      << "\n";
            any_code_processed = true;
        }

        if (!any_code_processed) {
            std::cerr << "[" << factor_name << "] no eligible codes processed.\n";
        } else {
            std::cout << "[" << factor_name << "] outputs in " << factor_out_dir.string() << "\n";
        }
    }

    if (!any_factor_run) {
        std::cerr << "No valid factor binding requested.\n";
        return 2;
    }

    std::cout << "Done. Outputs in: " << out_dir_base << "\n";
    return 0;
}
