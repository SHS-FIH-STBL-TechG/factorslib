#include "factor_leverage_optimizer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <unordered_map>
#include <utility>

#include <boost/math/distributions/normal.hpp>

namespace factorlib::tools {

namespace {

inline double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

double quantile_sorted(const std::vector<double>& sorted, double p) {
    if (sorted.empty()) return std::numeric_limits<double>::quiet_NaN();
    double pp = clamp(p, 0.0, 1.0);
    double idx = pp * (sorted.size() - 1);
    std::size_t i = static_cast<std::size_t>(std::floor(idx));
    std::size_t j = std::min(i + 1, sorted.size() - 1);
    double t = idx - static_cast<double>(i);
    return sorted[i] * (1.0 - t) + sorted[j] * t;
}

double compute_median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    double m = v[v.size() / 2];
    if (v.size() % 2 == 0) {
        auto it = std::max_element(v.begin(), v.begin() + v.size() / 2);
        m = 0.5 * (m + *it);
    }
    return m;
}

FactorDistributionKind classify_kind(const FactorProfile& p, const LeverageSearchConfig& cfg) {
    const bool is_discrete = (p.unique_ratio < cfg.discrete_unique_ratio_threshold) ||
                             (p.max_freq_ratio > cfg.discrete_max_freq_ratio_threshold);
    if (is_discrete) return FactorDistributionKind::DiscreteOrNonContinuous;
    if (p.symmetry_score <= cfg.symmetry_score_threshold) return FactorDistributionKind::SymmetricContinuous;
    return FactorDistributionKind::Asymmetric;
}

struct Candidate {
    TradeMode mode;
    int polarity;
};

std::vector<Candidate> candidates_for_kind(FactorDistributionKind kind) {
    std::vector<Candidate> out;
    auto add = [&](TradeMode m) {
        out.push_back({m, +1});
        out.push_back({m, -1});
    };
    if (kind == FactorDistributionKind::SymmetricContinuous) {
        add(TradeMode::BothSides);
        return out;
    }
    if (kind == FactorDistributionKind::Asymmetric) {
        add(TradeMode::PositiveOnly);
        add(TradeMode::NegativeOnly);
        return out;
    }
    add(TradeMode::BothSides);
    add(TradeMode::PositiveOnly);
    add(TradeMode::NegativeOnly);
    return out;
}

} // namespace

FactorLeverageOptimizer::FactorLeverageOptimizer(LeverageSearchConfig cfg) : _cfg(cfg) {}

double FactorLeverageOptimizer::finalize_leverage(double leverage, double raw_signal) const {
    // 仅做最大杠杆裁剪：不再把 |L|<1 强制抬到 1
    if (raw_signal == 0.0) {
        return 0.0;
    }
    if (leverage > _cfg.max_leverage) leverage = _cfg.max_leverage;
    if (leverage < -_cfg.max_leverage) leverage = -_cfg.max_leverage;
    return leverage;
}

FactorProfile FactorLeverageOptimizer::analyze_profile(const std::vector<double>& x) const {
    FactorProfile p;
    if (x.empty()) {
        p.kind = FactorDistributionKind::DiscreteOrNonContinuous;
        return p;
    }

    p.median = compute_median(x);

    constexpr double eps = 1e-6;

    std::unordered_map<long long, std::size_t> freq;
    freq.reserve(x.size() * 2);

    std::vector<double> centered;
    centered.reserve(x.size());
    for (double v : x) {
        double c = v - p.median;
        centered.push_back(c);
        long long key = static_cast<long long>(std::llround(c / eps));
        freq[key] += 1;
    }

    p.unique_ratio = static_cast<double>(freq.size()) / static_cast<double>(x.size());
    std::size_t maxf = 0;
    for (const auto& kv : freq) maxf = std::max(maxf, kv.second);
    p.max_freq_ratio = static_cast<double>(maxf) / static_cast<double>(x.size());

    std::vector<double> sorted = centered;
    std::sort(sorted.begin(), sorted.end());
    double q10 = quantile_sorted(sorted, 0.10);
    double q50 = quantile_sorted(sorted, 0.50);
    double q90 = quantile_sorted(sorted, 0.90);
    double denom = std::max(1e-12, q90 - q10);
    p.symmetry_score = std::abs(0.5 * (q90 + q10) - q50) / denom;

    p.kind = classify_kind(p, _cfg);
    return p;
}

std::vector<double> FactorLeverageOptimizer::rank_normalize_to_z(const std::vector<double>& x_centered) const {
    const std::size_t n = x_centered.size();
    std::vector<double> z(n, 0.0);
    if (n == 0) return z;

    std::vector<std::pair<double, std::size_t>> items;
    items.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        items.emplace_back(x_centered[i], i);
    }
    std::sort(items.begin(), items.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    boost::math::normal_distribution<double> norm;

    std::size_t i = 0;
    while (i < n) {
        std::size_t j = i + 1;
        while (j < n && items[j].first == items[i].first) {
            ++j;
        }
        double rank_mid = 0.5 * (static_cast<double>(i + 1) + static_cast<double>(j));
        double u = (rank_mid - 0.5) / static_cast<double>(n);
        u = clamp(u, 1e-6, 1.0 - 1e-6);
        double q = boost::math::quantile(norm, u);

        for (std::size_t k = i; k < j; ++k) {
            z[items[k].second] = q;
        }
        i = j;
    }
    return z;
}

ThresholdSearchResult FactorLeverageOptimizer::search_best_threshold(const std::vector<int64_t>& ts,
                                                                     const std::vector<double>& x_raw,
                                                                     const std::vector<double>& z,
                                                                     const std::vector<double>& next_ret,
                                                                     const std::vector<double>& full_next_ret,
                                                                     int D_sample_days) const {
    ThresholdSearchResult best;
    if (ts.size() != z.size() || z.size() != next_ret.size() || z.empty()) {
        return best;
    }

    best.profile = analyze_profile(x_raw);
    auto cand = candidates_for_kind(best.profile.kind);

    // 基准 score：不使用因子，使用裁剪后的全样本 next_ret（长度约等于 D-1），等价于 T=D 的 1x 持有。
    if (D_sample_days > 0 && !full_next_ret.empty()) {
        double E_base = 1.0;
        double peak = 1.0;
        double sum_dd2 = 0.0;
        for (double r : full_next_ret) {
            double term = 1.0 + r;
            if (!(term > 0.0) || !std::isfinite(term)) {
                continue;
            }
            E_base *= term;
            if (E_base > peak) peak = E_base;
            double dd = (peak > 0.0) ? (peak - E_base) / peak : 0.0;
            sum_dd2 += dd * dd;
        }
        double dd_rms_base = std::sqrt(sum_dd2 / std::max(1.0, static_cast<double>(D_sample_days)));
        if (dd_rms_base > 1e-12 && std::isfinite(dd_rms_base)) {
            best.baseline_score = (E_base - 1.0) / dd_rms_base;
        }
    }

    std::vector<double> sorted_raw = x_raw;
    std::sort(sorted_raw.begin(), sorted_raw.end());
    boost::math::normal_distribution<double> norm_cdf;

    auto apply_raw_thresholds = [&](ThresholdSearchResult& r) {
        r.theta_raw_low = std::numeric_limits<double>::quiet_NaN();
        r.theta_raw_high = std::numeric_limits<double>::quiet_NaN();
        if (sorted_raw.empty()) {
            return;
        }
        double p_high = clamp(boost::math::cdf(norm_cdf, r.theta), 0.0, 1.0);
        double p_low = clamp(1.0 - p_high, 0.0, 1.0);
        if (r.mode == TradeMode::BothSides || r.mode == TradeMode::NegativeOnly) {
            r.theta_raw_low = quantile_sorted(sorted_raw, p_low);
        }
        if (r.mode == TradeMode::BothSides || r.mode == TradeMode::PositiveOnly) {
            r.theta_raw_high = quantile_sorted(sorted_raw, p_high);
        }
    };

    std::vector<double> zabs;
    zabs.reserve(z.size());
    for (double v : z) zabs.push_back(std::abs(v));
    std::sort(zabs.begin(), zabs.end());
    double z_cap_global = quantile_sorted(zabs, _cfg.z_cap_quantile);
    z_cap_global = clamp(z_cap_global, _cfg.z_cap_min, _cfg.z_cap_max);

    // 年均交易日下限：trade_days / (D_sample_days/252) >= 50
    std::size_t min_trade_days = 0;
    if (D_sample_days > 0) {
        double years = static_cast<double>(D_sample_days) / 252.0;
        min_trade_days = static_cast<std::size_t>(std::ceil(years * 50.0 - 1e-12));
    }

    auto eval_one = [&](double theta, TradeMode mode, int polarity) -> std::optional<ThresholdSearchResult> {
        std::vector<double> b(z.size(), 0.0);
        double sum_b2 = 0.0;
        double max_abs_b = 0.0;
        std::size_t trades = 0;

        double z_cap = std::max(z_cap_global, theta + 1e-3);
        for (std::size_t t = 0; t < z.size(); ++t) {
            double zv = z[t];
            bool active = false;
            int sgn = 0;

            if (mode == TradeMode::BothSides) {
                if (std::abs(zv) > theta) {
                    active = true;
                    sgn = (zv > 0) ? +1 : -1;
                }
            } else if (mode == TradeMode::PositiveOnly) {
                if (zv > theta) {
                    active = true;
                    sgn = +1;
                }
            } else {
                if (zv < -theta) {
                    active = true;
                    sgn = -1;
                }
            }

            if (!active) {
                b[t] = 0.0;
                continue;
            }

            double a = std::abs(zv);
            double frac = (z_cap > theta) ? (a - theta) / (z_cap - theta) : 0.0;
            frac = clamp(frac, 0.0, 1.0);
            double mag = 1.0 + frac * (_cfg.max_leverage - 1.0);

            double bt = static_cast<double>(polarity * sgn) * mag;
            b[t] = bt;

            sum_b2 += bt * bt;
            max_abs_b = std::max(max_abs_b, std::abs(bt));
            trades += 1;
        }

        if (trades < min_trade_days) {
            return std::nullopt;
        }

        if (sum_b2 <= 1e-12 || max_abs_b <= 1e-12) {
            return std::nullopt;
        }

        double c_risk = std::sqrt(static_cast<double>(D_sample_days) / sum_b2);
        double c_max = _cfg.max_leverage / max_abs_b;
        double c = std::min(c_risk, c_max);

        std::vector<double> equity(z.size(), 1.0);
        double E = 1.0;
        double peak = 1.0;
        double sum_dd2 = 0.0;

        for (std::size_t t = 0; t < z.size(); ++t) {
            double L = c * b[t];
            L = finalize_leverage(L, b[t]);

            double term = 1.0 + L * next_ret[t];
            if (!(term > 0.0) || !std::isfinite(term)) {
                return std::nullopt;
            }
            E *= term;
            equity[t] = E;

            if (E > peak) peak = E;
            double dd = (peak > 0.0) ? (peak - E) / peak : 0.0;
            sum_dd2 += dd * dd;
        }

        double dd_rms = std::sqrt(sum_dd2 / std::max(1.0, static_cast<double>(D_sample_days)));
        if (!(dd_rms > 1e-12) || !std::isfinite(dd_rms)) {
            return std::nullopt;
        }

        double score = (E - 1.0) / dd_rms;

        ThresholdSearchResult r;
        r.ok = true;
        r.theta = theta;
        r.score = score;
        r.final_equity = E;
        r.dd_rms = dd_rms;
        r.c_scale = c;
        r.z_cap = std::max(z_cap_global, theta + 1e-3);
        r.mode = mode;
        r.polarity = polarity;
        r.trade_days = trades;
        r.D_sample_days = D_sample_days;
        r.T_trade_days = static_cast<int>(z.size());
        r.profile = best.profile;
        r.baseline_score = best.baseline_score;
        return r;
    };

    for (double theta = _cfg.theta_min; theta <= _cfg.theta_max + 1e-12; theta += _cfg.theta_step) {
        for (const auto& c : cand) {
            auto r = eval_one(theta, c.mode, c.polarity);
            if (!r.has_value()) continue;
            const auto& rr = r.value();
            if (!best.ok || rr.score > best.score ||
                (std::abs(rr.score - best.score) < 1e-12 && rr.theta < best.theta)) {
                best = rr;
            }
        }
    }

    if (best.ok) {
        apply_raw_thresholds(best);
    }

    return best;
}

std::vector<LeveragePoint> FactorLeverageOptimizer::build_leverage_series(const std::vector<int64_t>& ts,
                                                                          const std::vector<double>& x_raw,
                                                                          const std::vector<double>& z,
                                                                          const std::vector<double>& next_ret,
                                                                          int D_sample_days,
                                                                          const ThresholdSearchResult& best) const {
    std::vector<LeveragePoint> out;
    if (!best.ok || ts.size() != z.size() || z.size() != next_ret.size()) return out;

    std::vector<double> zabs;
    zabs.reserve(z.size());
    for (double v : z) zabs.push_back(std::abs(v));
    std::sort(zabs.begin(), zabs.end());
    double z_cap_global = quantile_sorted(zabs, _cfg.z_cap_quantile);
    z_cap_global = clamp(z_cap_global, _cfg.z_cap_min, _cfg.z_cap_max);
    double z_cap = std::max(z_cap_global, best.theta + 1e-3);

    std::vector<double> b(z.size(), 0.0);
    double sum_b2 = 0.0;
    double max_abs_b = 0.0;

    for (std::size_t t = 0; t < z.size(); ++t) {
        double zv = z[t];
        bool active = false;
        int sgn = 0;

        if (best.mode == TradeMode::BothSides) {
            if (std::abs(zv) > best.theta) {
                active = true;
                sgn = (zv > 0) ? +1 : -1;
            }
        } else if (best.mode == TradeMode::PositiveOnly) {
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

        if (!active) {
            b[t] = 0.0;
            continue;
        }

        double a = std::abs(zv);
        double frac = (z_cap > best.theta) ? (a - best.theta) / (z_cap - best.theta) : 0.0;
        frac = clamp(frac, 0.0, 1.0);
        double mag = 1.0 + frac * (_cfg.max_leverage - 1.0);

        double bt = static_cast<double>(best.polarity * sgn) * mag;
        b[t] = bt;

        sum_b2 += bt * bt;
        max_abs_b = std::max(max_abs_b, std::abs(bt));
    }

    double c_risk = std::sqrt(static_cast<double>(D_sample_days) / std::max(1e-12, sum_b2));
    double c_max = _cfg.max_leverage / std::max(1e-12, max_abs_b);
    double c = std::min(c_risk, c_max);

    out.reserve(z.size());
    double E = 1.0;
    double peak = 1.0;

    for (std::size_t t = 0; t < z.size(); ++t) {
        LeveragePoint p;
        p.ts_ms = ts[t];
        p.x_raw = (t < x_raw.size() ? x_raw[t] : 0.0);
        p.z = z[t];
        p.b_raw = b[t];
        p.active = (b[t] != 0.0);

        double L = c * b[t];
        L = finalize_leverage(L, b[t]);
        p.leverage = L;
        p.ret = next_ret[t];

        double term = 1.0 + L * next_ret[t];
        if (term > 0.0 && std::isfinite(term)) {
            E *= term;
        }
        p.equity = E;
        if (E > peak) peak = E;
        p.drawdown = (peak > 0.0) ? (peak - E) / peak : 0.0;

        out.push_back(p);
    }

    return out;
}

} // namespace factorlib::tools
