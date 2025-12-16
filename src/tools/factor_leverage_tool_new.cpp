#include "factor_leverage_optimizer.h"
#include "factor_leverage_tool_config.h"
#include "kline_csv_loader.h"
#include "sliding_gaussian_leverage.h"

#include "core/databus.h"
#include "core/types.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <cstdio>
#include <cstring>
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
#include <system_error>
#include <thread>
#include <tuple>
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

#if !defined(_WIN32)
std::optional<std::string> maybe_default_linux_parquet_dir() {
    static const char* kDefault = "/mnt/disk2/qdata/dm/401000001";
    std::error_code ec;
    if (fs::exists(kDefault, ec) && fs::is_directory(kDefault, ec)) {
        return std::string(kDefault);
    }
    return std::nullopt;
}

int64_t make_time_ms_utc_close(int year, int month, int day, int hour = 15) {
    std::tm tm{};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = 0;
    tm.tm_sec = 0;
    return static_cast<int64_t>(timegm(&tm)) * 1000;
}

int64_t ts_from_yyyymmdd(int yyyymmdd) {
    int year = yyyymmdd / 10000;
    int month = (yyyymmdd / 100) % 100;
    int day = yyyymmdd % 100;
    return make_time_ms_utc_close(year, month, day);
}

static inline uint16_t read_u16_le(const unsigned char* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

static inline int32_t read_i32_le(const unsigned char* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) |
                                (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) |
                                (static_cast<uint32_t>(p[3]) << 24));
}

static inline double read_f64_le(const unsigned char* p) {
    static_assert(sizeof(double) == 8, "double must be 8 bytes");
    double v = 0.0;
    std::memcpy(&v, p, 8);
    return v;
}

std::vector<KlineCsvSeries> load_kline_series_from_parquet(const fs::path& parquet_path,
                                                           const fs::path& out_dir_base,
                                                           const std::vector<std::string>& codes_filter,
                                                           const std::optional<int64_t>& start_ms,
                                                           const std::optional<int64_t>& end_ms) {
    std::ostringstream tag;
    tag << "parquet_stream_" << std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count();
    fs::path cache_dir = out_dir_base / tag.str();
    fs::create_directories(cache_dir);

    fs::path script_path = cache_dir / "parquet_to_kline_stream.py";
    {
        std::ofstream script(script_path);
        script << R"PY(import sys, struct

def _pick(cols, candidates):
    cols_l = {c.lower(): c for c in cols}
    for name in candidates:
        if name.lower() in cols_l:
            return cols_l[name.lower()]
    return None

def _to_int_yyyymmdd(v):
    if v is None:
        return None
    try:
        import pandas as pd
        if isinstance(v, pd.Timestamp):
            return int(v.strftime("%Y%m%d"))
    except Exception:
        pass
    try:
        if isinstance(v, (int,)) or (isinstance(v, str) and v.isdigit()):
            s = str(v)
            if len(s) == 8:
                return int(s)
    except Exception:
        pass
    try:
        s = str(v)
        if len(s) >= 10 and s[4] == "-" and s[7] == "-":
            return int(s[0:4] + s[5:7] + s[8:10])
    except Exception:
        pass
    return None

def _scale_price(x):
    if x is None:
        return None
    try:
        # dm/401000001 px* is uint32 with 2 decimals, stored as integer points
        if isinstance(x, (int,)) or (isinstance(x, str) and x.isdigit()):
            xi = int(x)
            return xi / 100.0
        return float(x)
    except Exception:
        return None

def _scale_qty_to_wangu(x):
    if x is None:
        return None
    try:
        # qty: 手; tests/data expects 成交量(万股). 1手=100股 => 万股=10000股 => 手*0.01
        return float(x) * 0.01
    except Exception:
        return None

def _scale_amt_to_wanyuan(x):
    if x is None:
        return None
    try:
        # amt: 千元; tests/data expects 成交额(万元) => /10
        return float(x) / 10.0
    except Exception:
        return None

def _ms_to_int_yyyymmdd(ms):
    from datetime import datetime, timezone, timedelta
    CST = timezone(timedelta(hours=8))
    dt = datetime.fromtimestamp(ms / 1000.0, tz=CST)
    return int(dt.strftime("%Y%m%d"))

def main():
    # argv: parquet_path codes_csv start_ms end_ms
    if len(sys.argv) < 2:
        print("Usage: python parquet_to_kline_stream.py parquet_path [codes_csv] [start_ms] [end_ms]", file=sys.stderr)
        return 2
    parquet_path = sys.argv[1]
    codes_csv = sys.argv[2] if len(sys.argv) > 2 else ""
    start_ms = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] else None
    end_ms = int(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[4] else None
    codes = set([c for c in codes_csv.split(",") if c]) if codes_csv else None

    try:
        import pyarrow as pa
        import pyarrow.dataset as ds
    except Exception as e:
        print("[ERR] pyarrow is required to read parquet:", e, file=sys.stderr)
        return 3

    dataset = ds.dataset(parquet_path, format="parquet")
    schema_cols = [f.name for f in dataset.schema]

    col_date = _pick(schema_cols, ["tradeDate", "trade_date", "date", "datetime", "dt", "time", "timestamp", "ts", "9"])
    col_code = _pick(schema_cols, ["icode", "code", "instrument_id", "symbol", "ticker", "windcode", "13", "gkid13", "gkid_13"])
    col_open = _pick(schema_cols, ["pxopen", "open", "open_price", "3000002"])
    col_high = _pick(schema_cols, ["pxhigh", "high", "high_price", "3000003"])
    col_low  = _pick(schema_cols, ["pxlow", "low", "low_price", "3000004"])
    col_close = _pick(schema_cols, ["pxclose", "close", "close_price", "adj_close", "price", "3000005"])
    col_qty = _pick(schema_cols, ["qty", "volume", "vol", "3000008"])
    col_amt = _pick(schema_cols, ["amt", "amount", "turnover", "3000009"])

    if not col_date or not col_close:
        print("[ERR] cannot find required columns in parquet schema:", schema_cols, file=sys.stderr)
        return 4

    columns = [c for c in [col_code, col_date, col_open, col_high, col_low, col_close, col_qty, col_amt] if c]

    filt = None
    try:
        if codes is not None and col_code:
            filt = ds.field(col_code).isin(list(codes))
        if col_date and (start_ms is not None or end_ms is not None):
            ftype = dataset.schema.field(col_date).type
            if pa.types.is_integer(ftype):
                lo = _ms_to_int_yyyymmdd(start_ms) if start_ms is not None else None
                hi = _ms_to_int_yyyymmdd(end_ms) if end_ms is not None else None
                if lo is not None:
                    expr = ds.field(col_date) >= lo
                    filt = expr if filt is None else (filt & expr)
                if hi is not None:
                    expr = ds.field(col_date) <= hi
                    filt = expr if filt is None else (filt & expr)
    except Exception:
        filt = filt

    scanner = dataset.scanner(columns=columns, filter=filt) if filt is not None else dataset.scanner(columns=columns)

    # Binary stream protocol (little-endian):
    #   magic 'FPL1'
    #   repeated records:
    #     u16 code_len, bytes[code_len] code_utf8
    #     i32 yyyymmdd
    #     f64 open, high, low, close
    #     f64 volume_wangu (can be NaN)
    #     f64 turnover_wanyuan (can be NaN)
    out = sys.stdout.buffer
    out.write(b"FPL1")

    for batch in scanner.to_batches():
        cols = batch.to_pydict()
        codes_col = cols.get(col_code, None) if col_code else None
        dates_col = cols.get(col_date, None)
        o_col = cols.get(col_open, None)
        h_col = cols.get(col_high, None)
        l_col = cols.get(col_low, None)
        c_col = cols.get(col_close, None)
        q_col = cols.get(col_qty, None)
        a_col = cols.get(col_amt, None)

        n = len(dates_col)
        for i in range(n):
            code = str(codes_col[i]) if codes_col is not None else "ALL"
            if codes is not None and code not in codes:
                continue
            ymd = _to_int_yyyymmdd(dates_col[i])
            if ymd is None:
                continue
            o = _scale_price(o_col[i]) if o_col is not None else None
            h = _scale_price(h_col[i]) if h_col is not None else None
            l = _scale_price(l_col[i]) if l_col is not None else None
            c = _scale_price(c_col[i])
            if c is None:
                continue
            vol = _scale_qty_to_wangu(q_col[i]) if q_col is not None else None
            amt = _scale_amt_to_wanyuan(a_col[i]) if a_col is not None else None

            code_b = code.encode("utf-8")
            if len(code_b) > 65535:
                continue
            out.write(struct.pack("<H", len(code_b)))
            out.write(code_b)
            out.write(struct.pack("<i", int(ymd)))
            out.write(struct.pack("<dddd", float(o) if o is not None else float(c),
                                  float(h) if h is not None else float(c),
                                  float(l) if l is not None else float(c),
                                  float(c)))
            out.write(struct.pack("<d", float(vol) if vol is not None else float("nan")))
            out.write(struct.pack("<d", float(amt) if amt is not None else float("nan")))

    return 0

if __name__ == "__main__":
    sys.exit(main())
)PY";
    }

    std::ostringstream codes_csv;
    for (std::size_t i = 0; i < codes_filter.size(); ++i) {
        if (i) codes_csv << ",";
        codes_csv << codes_filter[i];
    }

    auto run_py = [&](const std::string& pybin) -> std::pair<FILE*, std::string> {
        std::ostringstream cmd;
        cmd << pybin << " \"" << script_path.string() << "\" \"" << parquet_path.string() << "\" \""
            << codes_csv.str() << "\" \"" << (start_ms ? std::to_string(*start_ms) : std::string()) << "\" \""
            << (end_ms ? std::to_string(*end_ms) : std::string()) << "\"";
        return {popen(cmd.str().c_str(), "r"), cmd.str()};
    };

    FILE* pipe = nullptr;
    std::string last_cmd;
    std::tie(pipe, last_cmd) = run_py("python3");
    if (!pipe) {
        std::tie(pipe, last_cmd) = run_py("python");
    }
    if (!pipe) {
        throw std::runtime_error("failed to start python to read parquet");
    }

    std::vector<unsigned char> buf;
    buf.reserve(1 << 20);
    unsigned char tmp[1 << 15];
    while (true) {
        std::size_t n = std::fread(tmp, 1, sizeof(tmp), pipe);
        if (n == 0) break;
        buf.insert(buf.end(), tmp, tmp + n);
    }
    int rc = pclose(pipe);
    if (rc != 0) {
        throw std::runtime_error("parquet stream failed (rc=" + std::to_string(rc) + "): " + last_cmd);
    }

    if (buf.size() < 4 || std::memcmp(buf.data(), "FPL1", 4) != 0) {
        throw std::runtime_error("invalid parquet stream header");
    }

    std::unordered_map<std::string, KlineCsvSeries> series_map;
    std::size_t off = 4;
    while (off < buf.size()) {
        if (off + 2 > buf.size()) {
            throw std::runtime_error("truncated parquet stream (code_len)");
        }
        uint16_t code_len = read_u16_le(buf.data() + off);
        off += 2;
        if (off + code_len > buf.size()) {
            throw std::runtime_error("truncated parquet stream (code)");
        }
        std::string code(reinterpret_cast<const char*>(buf.data() + off), code_len);
        off += code_len;
        if (off + 4 + 8 * 6 > buf.size()) {
            throw std::runtime_error("truncated parquet stream (payload)");
        }
        int32_t ymd = read_i32_le(buf.data() + off);
        off += 4;
        double open = read_f64_le(buf.data() + off);
        off += 8;
        double high = read_f64_le(buf.data() + off);
        off += 8;
        double low = read_f64_le(buf.data() + off);
        off += 8;
        double close = read_f64_le(buf.data() + off);
        off += 8;
        double vol = read_f64_le(buf.data() + off);
        off += 8;
        double amt = read_f64_le(buf.data() + off);
        off += 8;

        factorlib::Bar bar{};
        bar.instrument_id = code;
        bar.data_time_ms = ts_from_yyyymmdd(ymd);
        bar.open = open;
        bar.high = high;
        bar.low = low;
        bar.close = close;
        bar.volume = std::isfinite(vol) ? static_cast<uint64_t>(vol) : 0;
        bar.turnover = std::isfinite(amt) ? amt : 0.0;
        bar.interval_ms = 24 * 60 * 60 * 1000;

        auto& s = series_map[code];
        if (s.code.empty()) {
            s.table_name = parquet_path.filename().string();
            s.code = code;
        }
        s.bars.push_back(bar);
    }

    std::vector<KlineCsvSeries> out;
    out.reserve(series_map.size());
    for (auto& kv : series_map) {
        auto& s = kv.second;
        std::sort(s.bars.begin(), s.bars.end(),
                  [](const factorlib::Bar& a, const factorlib::Bar& b) { return a.data_time_ms < b.data_time_ms; });
        if (!s.bars.empty()) out.emplace_back(std::move(s));
    }
    std::sort(out.begin(), out.end(), [](const KlineCsvSeries& a, const KlineCsvSeries& b) { return a.code < b.code; });
    return out;
}

#endif

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
    sec -= 8 * 3600;
    return static_cast<int64_t>(sec) * 1000;
}

std::string format_date_utc(int64_t ms) {
    std::time_t sec = static_cast<std::time_t>(ms / 1000) + 8 * 3600;
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

struct WfPoint {
    int64_t ts_ms = 0;
    double x_raw = std::numeric_limits<double>::quiet_NaN();
    double z = std::numeric_limits<double>::quiet_NaN();
    // policy (fit on trailing window)
    factorlib::tools::TradeMode mode = factorlib::tools::TradeMode::BothSides;
    int polarity = +1;
    double theta = 0.0;
    double z_cap = 0.0;
    double c_scale = 0.0;
    // action
    double b_raw = 0.0;
    double leverage = 0.0;
    // realized
    double ret = 0.0;
    double equity = 1.0;
    double drawdown = 0.0;
    bool active = false;
    // diagnostics from training window
    factorlib::tools::FactorDistributionKind profile_kind = factorlib::tools::FactorDistributionKind::DiscreteOrNonContinuous;
    double symmetry_score = std::numeric_limits<double>::quiet_NaN();
    double unique_ratio = std::numeric_limits<double>::quiet_NaN();
    double max_freq_ratio = std::numeric_limits<double>::quiet_NaN();
    double theta_raw_low = std::numeric_limits<double>::quiet_NaN();
    double theta_raw_high = std::numeric_limits<double>::quiet_NaN();
};

double compute_b_raw(double zv,
                     const factorlib::tools::ThresholdSearchResult& best,
                     const factorlib::tools::LeverageSearchConfig& cfg) {
    bool active = false;
    int sgn = 0;

    // 注意：zv 为 NaN 时，所有比较为 false => active=false => 返回 0 (空仓)
    if (best.mode == factorlib::tools::TradeMode::BothSides) {
        if (std::abs(zv) > best.theta) {
            active = true;
            sgn = (zv > 0) ? +1 : -1;
        }
    } else if (best.mode == factorlib::tools::TradeMode::PositiveOnly) {
        if (zv > best.theta) {
            active = true;
            sgn = +1;
        }
    } else {
        if (zv < -best.theta) {
            active = true;
            sgn = -1;
        }
    }
    if (!active) return 0.0;

    double z_cap = std::max(best.z_cap, best.theta + 1e-3);
    double a = std::abs(zv);
    double frac = (z_cap > best.theta) ? (a - best.theta) / (z_cap - best.theta) : 0.0;
    frac = std::clamp(frac, 0.0, 1.0);
    double mag = 1.0 + frac * (cfg.max_leverage - 1.0);
    return static_cast<double>(best.polarity * sgn) * mag;
}

// Same behavior as FactorLeverageOptimizer::finalize_leverage (but that method is private).
// We duplicate it here to keep this tool self-contained and compilable.
static inline double finalize_leverage_like_optimizer(double leverage,
                                                     double raw_signal,
                                                     const factorlib::tools::LeverageSearchConfig& cfg) {
    // Only max leverage clamp; do NOT force |L| >= 1.
    if (raw_signal == 0.0 || !std::isfinite(raw_signal)) {
        return 0.0;
    }
    if (leverage > cfg.max_leverage) leverage = cfg.max_leverage;
    if (leverage < -cfg.max_leverage) leverage = -cfg.max_leverage;
    return leverage;
}

void write_wf_csv(const fs::path& path,
                  const std::vector<WfPoint>& pts,
                  const std::string& code,
                  const std::string& factor_key) {
    std::ofstream out(path);
    out << "date,code,factor,factor_raw,z,mode,polarity,theta,z_cap,c_scale,b_raw,leverage,next_return,equity,drawdown,active,profile_kind,symmetry_score,unique_ratio,max_freq_ratio,raw_threshold_low,raw_threshold_high\n";
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
            << mode_to_str(p.mode) << ","
            << p.polarity << ","
            << p.theta << ","
            << p.z_cap << ","
            << p.c_scale << ","
            << p.b_raw << ","
            << p.leverage << ","
            << p.ret << ","
            << p.equity << ","
            << p.drawdown << ","
            << (p.active ? 1 : 0) << ","
            << kind_to_str(p.profile_kind) << ","
            << p.symmetry_score << ","
            << p.unique_ratio << ","
            << p.max_freq_ratio << ","
            << p.theta_raw_low << ","
            << p.theta_raw_high
            << "\n";
    }
}

void append_wf_summary(const fs::path& path,
                       const std::string& code,
                       const std::string& factor_key,
                       int train_window,
                       int step,
                       std::size_t zwin,
                       double oos_score,
                       double oos_baseline_score,
                       double final_equity,
                       double dd_rms,
                       std::size_t trade_days,
                       std::size_t T_oos_days,
                       double avg_theta,
                       double avg_c_scale) {
    const bool exists = fs::exists(path);
    std::ofstream out(path, std::ios::app);
    if (!exists) {
        out << "code,factor,oos_score,oos_baseline_score,train_window,step,zwin,final_equity,dd_rms,trade_days,T_oos_days,avg_theta,avg_c_scale\n";
    }
    out << code << ","
        << factor_key << ","
        << oos_score << ","
        << oos_baseline_score << ","
        << train_window << ","
        << step << ","
        << zwin << ","
        << final_equity << ","
        << dd_rms << ","
        << trade_days << ","
        << T_oos_days << ","
        << avg_theta << ","
        << avg_c_scale
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
#if !defined(_WIN32)
    (void)script_path;
    (void)csv_path;
    (void)png_path;
    (void)title;
    (void)factor_col;
    (void)b_col;
    (void)leverage_col;
    (void)theta_text;
    (void)score_text;
    (void)baseline_text;
    return false;
#else
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
    # argv: data.csv out.png title factor_col b_col leverage_col theta_text score_text baseline_text
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
#endif
}

} // namespace

int main(int argc, char** argv) {
    if (has_flag(argc, argv, "--help")) {
        std::cout
            << "Usage:\n"
            << "  factor_leverage_tool_new \\\n"
            << "    [--data_dir DIR]                (default tests/data) \\\n"
            << "    [--parquet_dir PATH]            (Linux only; direct read parquet via pyarrow) \\\n"
            << "    [--factor name1,name2|all]      (default all built-ins) \\\n"
            << "    [--codes CODE1,CODE2,...]       (default load every CSV) \\\n"
            << "    [--start YYYY-MM-DD] [--end YYYY-MM-DD] \\\n"
            << "    [--out_dir DIR]                 (default output/factor_leverage) \\\n"
            << "    [--theta_min 0] [--theta_max 2.5] [--theta_step 0.05] \\\n"
            << "    [--D 250] [--max_leverage 2] \\\n"
            << "    [--train 252] [--step 1] [--zwin 250]\n"
#if !defined(_WIN32)
            << "Notes:\n"
            << "  - On Linux, this tool does not generate PNG plots.\n"
#endif
            ;
        return 0;
    }

    std::string data_dir = get_arg(argc, argv, "--data_dir").value_or(DefaultDataDir());
    std::optional<int64_t> start_ms;
    std::optional<int64_t> end_ms;
    if (auto v = get_arg(argc, argv, "--start")) start_ms = parse_date_utc_ms(*v);
    if (auto v = get_arg(argc, argv, "--end")) end_ms = parse_date_utc_ms(*v);

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
 
#if !defined(_WIN32)
    std::optional<std::string> parquet_dir = get_arg(argc, argv, "--parquet_dir");
    if (!parquet_dir.has_value() && !get_arg(argc, argv, "--data_dir").has_value()) {
        parquet_dir = maybe_default_linux_parquet_dir();
    }
#endif

    factorlib::tools::LeverageSearchConfig cfg;
    if (auto v = get_arg(argc, argv, "--theta_min")) cfg.theta_min = std::stod(*v);
    if (auto v = get_arg(argc, argv, "--theta_max")) cfg.theta_max = std::stod(*v);
    if (auto v = get_arg(argc, argv, "--theta_step")) cfg.theta_step = std::stod(*v);
    bool D_overridden = false;
    if (auto v = get_arg(argc, argv, "--D")) { cfg.D = std::stoi(*v); D_overridden = true; }
    if (auto v = get_arg(argc, argv, "--max_leverage")) cfg.max_leverage = std::stod(*v);

    // walk-forward (no-leak) settings
    const int train_window = std::max(20, std::stoi(get_arg(argc, argv, "--train").value_or("252")));
    const int step = std::max(1, std::stoi(get_arg(argc, argv, "--step").value_or("1")));
    const std::size_t zwin = static_cast<std::size_t>(std::max(5, std::stoi(get_arg(argc, argv, "--zwin").value_or("250"))));

    factorlib::tools::FactorLeverageOptimizer opt(cfg);

    std::vector<factorlib::tools::KlineCsvSeries> series_list;
#if !defined(_WIN32)
    if (parquet_dir.has_value()) {
        series_list = load_kline_series_from_parquet(*parquet_dir, out_dir_base, codes, start_ms, end_ms);
    } else
#endif
    {
        factorlib::tools::KlineCsvLoader loader(data_dir);
        series_list = loader.load(codes, start_ms, end_ms);
    }
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
#if defined(_WIN32)
        fs::path plot_script = factor_out_dir / "plot_distribution.py";
#endif

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

            // === 全日历评估：以 returns 为主轴（不再按 factor_pts 对齐后 continue 删日历） ===
            auto rets = compute_next_returns(bars);
            if (rets.size() < 5) {
                std::cerr << "[" << factor_name << "] skip " << code << " (too few returns)\n";
                continue;
            }

            std::vector<int64_t> ts;
            std::vector<double> next_ret;
            ts.reserve(rets.size());
            next_ret.reserve(rets.size());
            for (const auto& rp : rets) {
                ts.push_back(rp.ts_ms);
                next_ret.push_back(rp.r);
            }

            std::mutex mtx;
            std::vector<std::pair<int64_t, double>> factor_pts;
            factor_pts.reserve(bars.size());

            factorlib::DataBus::instance().subscribe<double>(
                binding.input_topic,
                code,
                [&mtx, &factor_pts](const std::string&, int64_t ts_ms, const double& v) {
                    std::lock_guard<std::mutex> lk(mtx);
                    factor_pts.emplace_back(ts_ms, v);
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

            // === 全日历回填 x_raw：缺失日 NaN（后续 => z=NaN => b=0 => 空仓） ===
            std::vector<double> x_raw(ts.size(), std::numeric_limits<double>::quiet_NaN());
            {
                std::lock_guard<std::mutex> lk(mtx);
                std::sort(factor_pts.begin(), factor_pts.end(),
                          [](const auto& a, const auto& b) { return a.first < b.first; });
            }

            std::unordered_map<int64_t, double> factor_by_ts;
            {
                std::lock_guard<std::mutex> lk(mtx);
                factor_by_ts.reserve(factor_pts.size() * 2);
                for (const auto& [t, x] : factor_pts) {
                    if (std::isfinite(x)) factor_by_ts[t] = x;
                }
            }

            int finite_cnt = 0;
            for (std::size_t i = 0; i < ts.size(); ++i) {
                auto it = factor_by_ts.find(ts[i]);
                if (it != factor_by_ts.end()) {
                    x_raw[i] = it->second;
                    if (std::isfinite(x_raw[i])) finite_cnt += 1;
                }
            }

            if (finite_cnt < 30) {
                std::cerr << "[" << factor_name << "] skip " << code << " (finite factor samples < 30)\n";
                continue;
            }

            // --- leakage-free z: online sliding rank->Gaussian
            // 缺失日（x_raw 为 NaN）不更新窗口，z 也置 NaN；交易端会自然变成空仓
            factorlib::tools::SlidingGaussianLeverage zg(zwin);
            std::vector<double> z;
            z.reserve(x_raw.size());
            for (double v : x_raw) {
                if (!std::isfinite(v)) {
                    z.push_back(std::numeric_limits<double>::quiet_NaN());
                    continue;
                }
                auto zo = zg.transform(v);
                z.push_back(zo.has_value() ? *zo : std::numeric_limits<double>::quiet_NaN());
            }

            if (static_cast<int>(ts.size()) <= train_window + 5) {
                std::cerr << "[" << factor_name << "] skip " << code
                          << " (samples too short for train=" << train_window << ")\n";
                continue;
            }

            // --- walk-forward: fit params on trailing window, evaluate EVERY day (full calendar)
            std::vector<WfPoint> wf_pts;
            wf_pts.reserve(ts.size() - static_cast<std::size_t>(train_window));

            std::optional<factorlib::tools::ThresholdSearchResult> last_good;

            double E = 1.0;
            double peak = 1.0;
            double sum_dd2 = 0.0;
            std::size_t trade_days = 0;

            double sum_theta = 0.0;
            double sum_c = 0.0;
            std::size_t cnt_policy = 0;
            double sum_xlow = 0.0;
            double sum_xhigh = 0.0;
            std::size_t cnt_xlow = 0;
            std::size_t cnt_xhigh = 0;

            for (std::size_t t = static_cast<std::size_t>(train_window); t < ts.size(); ++t) {
                // 每 step 天更新一次 policy（fit on [t-train_window, t)）
                if (((t - static_cast<std::size_t>(train_window)) % static_cast<std::size_t>(step)) == 0) {
                    const std::size_t s = t - static_cast<std::size_t>(train_window);
                    const std::size_t e = t;

                    std::vector<int64_t> ts_tr(ts.begin() + static_cast<std::ptrdiff_t>(s), ts.begin() + static_cast<std::ptrdiff_t>(e));
                    std::vector<double>  x_tr (x_raw.begin() + static_cast<std::ptrdiff_t>(s), x_raw.begin() + static_cast<std::ptrdiff_t>(e));
                    std::vector<double>  z_tr (z.begin() + static_cast<std::ptrdiff_t>(s), z.begin() + static_cast<std::ptrdiff_t>(e));
                    std::vector<double>  r_tr (next_ret.begin() + static_cast<std::ptrdiff_t>(s), next_ret.begin() + static_cast<std::ptrdiff_t>(e));

                    // baseline within the same train window
                    std::vector<double> full_tr = r_tr;
                    const int D_eff = static_cast<int>(r_tr.size());

                    auto best = opt.search_best_threshold(ts_tr, x_tr, z_tr, r_tr, full_tr, D_eff);
                    if (best.ok) {
                        last_good = best;
                    }
                }

                const factorlib::tools::ThresholdSearchResult* pol = last_good ? &(*last_good) : nullptr;

                WfPoint p;
                p.ts_ms = ts[t];
                p.x_raw = x_raw[t];
                p.z = z[t];
                p.ret = next_ret[t];

                double b_raw = 0.0;
                double L = 0.0;

                if (pol) {
                    p.mode = pol->mode;
                    p.polarity = pol->polarity;
                    p.theta = pol->theta;
                    p.z_cap = pol->z_cap;
                    p.c_scale = pol->c_scale;
                    p.profile_kind = pol->profile.kind;
                    p.symmetry_score = pol->profile.symmetry_score;
                    p.unique_ratio = pol->profile.unique_ratio;
                    p.max_freq_ratio = pol->profile.max_freq_ratio;
                    p.theta_raw_low = pol->theta_raw_low;
                    p.theta_raw_high = pol->theta_raw_high;

                    b_raw = compute_b_raw(p.z, *pol, cfg);
                    L = finalize_leverage_like_optimizer(p.c_scale * b_raw, b_raw, cfg);
                    p.active = (b_raw != 0.0);

                    // 统计策略参数均值（按“评估日历”计数）
                    sum_theta += p.theta;
                    sum_c += p.c_scale;
                    cnt_policy += 1;
                    if (std::isfinite(p.theta_raw_low)) {
                        sum_xlow += p.theta_raw_low;
                        cnt_xlow += 1;
                    }
                    if (std::isfinite(p.theta_raw_high)) {
                        sum_xhigh += p.theta_raw_high;
                        cnt_xhigh += 1;
                    }
                }

                p.b_raw = b_raw;
                p.leverage = L;

                // === 全日历累计：空仓日 L=0 => term=1；但 drawdown 可能持续 ===
                const double term = 1.0 + L * p.ret;
                if (term > 0.0 && std::isfinite(term)) {
                    E *= term;
                }
                p.equity = E;
                if (E > peak) peak = E;
                p.drawdown = (peak > 0.0) ? (peak - E) / peak : 0.0;

                sum_dd2 += p.drawdown * p.drawdown;
                if (p.active) trade_days += 1;

                wf_pts.push_back(p);
            }

            if (wf_pts.size() < 20) {
                std::cerr << "[" << factor_name << "] skip " << code << " (oos points < 20)\n";
                continue;
            }

            const double D_oos = static_cast<double>(wf_pts.size());
            const double dd_rms = std::sqrt(sum_dd2 / std::max(1.0, D_oos));
            const double oos_score = (dd_rms > 1e-12 && std::isfinite(dd_rms))
                ? ((E - 1.0) / dd_rms)
                : std::numeric_limits<double>::quiet_NaN();

            // baseline (1x hold) on the same oos slice (full calendar within OOS)
            double E_base = 1.0;
            double peak_base = 1.0;
            double sum_dd2_base = 0.0;
            for (const auto& pp : wf_pts) {
                const double term = 1.0 + pp.ret;
                if (term > 0.0 && std::isfinite(term)) E_base *= term;
                if (E_base > peak_base) peak_base = E_base;
                const double dd = (peak_base > 0.0) ? (peak_base - E_base) / peak_base : 0.0;
                sum_dd2_base += dd * dd;
            }
            const double dd_rms_base = std::sqrt(sum_dd2_base / std::max(1.0, D_oos));
            const double oos_baseline_score = (dd_rms_base > 1e-12 && std::isfinite(dd_rms_base))
                ? ((E_base - 1.0) / dd_rms_base)
                : std::numeric_limits<double>::quiet_NaN();

            const double avg_theta = (cnt_policy > 0) ? (sum_theta / static_cast<double>(cnt_policy)) : std::numeric_limits<double>::quiet_NaN();
            const double avg_c_scale = (cnt_policy > 0) ? (sum_c / static_cast<double>(cnt_policy)) : std::numeric_limits<double>::quiet_NaN();
            const double avg_theta_raw_low = (cnt_xlow > 0) ? (sum_xlow / static_cast<double>(cnt_xlow)) : std::numeric_limits<double>::quiet_NaN();
            const double avg_theta_raw_high = (cnt_xhigh > 0) ? (sum_xhigh / static_cast<double>(cnt_xhigh)) : std::numeric_limits<double>::quiet_NaN();

            fs::path csv_path = factor_out_dir / ("leverage_" + code + "_" + factor_name + ".csv");
            write_wf_csv(csv_path, wf_pts, code, factor_name);

            append_wf_summary(summary_path, code, factor_name,
                              train_window, step, zwin,
                              oos_score, oos_baseline_score,
                              E, dd_rms, trade_days, wf_pts.size(),
                              avg_theta, avg_c_scale);

            fs::path dist_png = factor_out_dir / ("distribution_" + code + "_" + factor_name + ".png");

            const bool should_plot = std::isfinite(oos_score) &&
                                     std::isfinite(oos_baseline_score) &&
                                     (oos_score >= oos_baseline_score);
#if defined(_WIN32)
            if (should_plot) {
                std::string plot_title = factor_name + " | " + code;
                std::ostringstream theta_stream;
                theta_stream << "walk-forward train=" << train_window
                             << ", step=" << step
                             << ", zwin=" << zwin
                             << ", theta_z_avg=" << avg_theta;
                if (std::isfinite(avg_theta_raw_high)) {
                    theta_stream << ", x_high_avg=" << avg_theta_raw_high;
                }
                if (std::isfinite(avg_theta_raw_low)) {
                    theta_stream << ", x_low_avg=" << avg_theta_raw_low;
                }
                std::string theta_label = theta_stream.str();
                std::ostringstream score_stream;
                score_stream << std::fixed << std::setprecision(4)
                             << "oos_score=" << oos_score;
                std::string score_label = score_stream.str();
                std::ostringstream baseline_stream;
                baseline_stream << std::fixed << std::setprecision(4)
                                << "oos_baseline=" << oos_baseline_score;
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
            } else {
                // 清理旧图，保证“只输出 oos_score >= baseline 的图表”
                std::error_code ec;
                fs::remove(dist_png, ec);
            }
#else
            (void)should_plot;
            // Linux: no plots
#endif

            std::cout << "[" << factor_name << "] OK " << code
                      << " oos_score=" << oos_score
                      << " oos_baseline=" << oos_baseline_score
                      << " train=" << train_window
                      << " step=" << step
                      << " zwin=" << zwin
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
