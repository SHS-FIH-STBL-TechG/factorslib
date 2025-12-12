#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <boost/math/distributions/normal.hpp>

namespace factorlib::tools {

/**
 * @brief 滑窗内对因子值做排名→正态化→输出剪裁的 z-score。
 *
 * - 按窗口内排序估计分位数；
 * - 使用标准正态分布的分位函数得到 z-score；
 * - 直接输出标准正态分位（可选在外部按需裁剪）。
 */
class SlidingGaussianLeverage {
public:
    struct DistributionSnapshot {
        std::size_t sample_count = 0;
        double min = std::numeric_limits<double>::quiet_NaN();
        double max = std::numeric_limits<double>::quiet_NaN();
        double mean = std::numeric_limits<double>::quiet_NaN();
        double std = std::numeric_limits<double>::quiet_NaN();
    };

    static constexpr std::size_t kDefaultWindow = 250;
    explicit SlidingGaussianLeverage(std::size_t window = kDefaultWindow)
        : _window_size(window ? window : kDefaultWindow) {}

    std::optional<double> transform(double value);

    const DistributionSnapshot& stats() const { return _stats; }

private:
    struct Sample {
        std::size_t seq;
        double value;
    };

    std::optional<double> compute_leverage();
    void update_stats();

    std::deque<Sample> _samples;
    std::size_t _next_seq = 0;
    std::size_t _window_size = kDefaultWindow;
    DistributionSnapshot _stats;
};

inline std::optional<double> SlidingGaussianLeverage::transform(double value) {
    if (!std::isfinite(value)) {
        return std::nullopt;
    }

    _samples.push_back(Sample{_next_seq++, value});
    while (_samples.size() > _window_size) {
        _samples.pop_front();
    }

    update_stats();
    return compute_leverage();
}

inline void SlidingGaussianLeverage::update_stats() {
    DistributionSnapshot snapshot;
    snapshot.sample_count = _samples.size();
    if (_samples.empty()) {
        _stats = snapshot;
        return;
    }

    double minv = std::numeric_limits<double>::infinity();
    double maxv = -std::numeric_limits<double>::infinity();
    double sum = 0.0;
    for (const auto& s : _samples) {
        minv = std::min(minv, s.value);
        maxv = std::max(maxv, s.value);
        sum += s.value;
    }
    snapshot.min = minv;
    snapshot.max = maxv;
    const double count = static_cast<double>(_samples.size());
    snapshot.mean = sum / count;

    double var_sum = 0.0;
    for (const auto& s : _samples) {
        const double diff = s.value - snapshot.mean;
        var_sum += diff * diff;
    }
    snapshot.std = std::sqrt(var_sum / count);
    _stats = snapshot;
}

inline std::optional<double> SlidingGaussianLeverage::compute_leverage() {
    if (_samples.empty()) {
        return std::nullopt;
    }

    std::vector<Sample> ranked(_samples.begin(), _samples.end());
    std::sort(ranked.begin(), ranked.end(), [](const Sample& a, const Sample& b) {
        if (std::isnan(a.value) && std::isnan(b.value)) return false;
        if (std::isnan(a.value)) return false;
        if (std::isnan(b.value)) return true;
        if (a.value == b.value) return a.seq < b.seq;
        return a.value < b.value;
    });

    const std::size_t target_seq = _samples.back().seq;
    std::size_t rank = ranked.size();
    for (std::size_t i = 0; i < ranked.size(); ++i) {
        if (ranked[i].seq == target_seq) {
            rank = i + 1;
            break;
        }
    }

    const double count = static_cast<double>(ranked.size());
    const double u = std::clamp((static_cast<double>(rank) - 0.5) / count,
                                1e-6,
                                1.0 - 1e-6);

    static const boost::math::normal normal_dist(0.0, 1.0);
    double z = boost::math::quantile(normal_dist, u);
    return z;
}

} // namespace factorlib::tools

