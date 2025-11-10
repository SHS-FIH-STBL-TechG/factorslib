// src/utils/math_utils.cpp
#include "utils/math_utils.h"
#include <numeric>
#include <algorithm>

namespace factorlib {

double MathUtils::mean(const std::vector<double>& data) {
    if (data.empty()) return 0.0;
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    return sum / data.size();
}

double MathUtils::stddev(const std::vector<double>& data) {
    if (data.size() < 2) return 0.0;
    double m = mean(data);
    double sum_sq = 0.0;
    for (double x : data) {
        sum_sq += (x - m) * (x - m);
    }
    return std::sqrt(sum_sq / (data.size() - 1));
}

double MathUtils::quantile(const std::vector<double>& data, double percentile) {
    if (data.empty()) return 0.0;

    std::vector<double> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());

    double position = percentile * (sorted_data.size() - 1);
    size_t index = static_cast<size_t>(std::floor(position));
    double fraction = position - index;

    if (index == sorted_data.size() - 1) {
        return sorted_data.back();
    } else {
        return sorted_data[index] + fraction * (sorted_data[index + 1] - sorted_data[index]);
    }
}

double MathUtils::correlation(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size() || x.size() < 2) return 0.0;

    double mean_x = mean(x);
    double mean_y = mean(y);
    double sum_xy = 0.0, sum_xx = 0.0, sum_yy = 0.0;

    for (size_t i = 0; i < x.size(); ++i) {
        double dx = x[i] - mean_x;
        double dy = y[i] - mean_y;
        sum_xy += dx * dy;
        sum_xx += dx * dx;
        sum_yy += dy * dy;
    }

    if (sum_xx == 0.0 || sum_yy == 0.0) return 0.0;
    return sum_xy / std::sqrt(sum_xx * sum_yy);
}

double MathUtils::normal_quantile(double p) {
    // 复用原来的Wichura算法实现
    if (p <= 0 || p >= 1) return 0.0;

    static const double a1 = -3.969683028665376e+01;
    // ... 完整的Wichura算法实现
    // [这里放置原来的normal_quantile函数实现]

    double q, r;
    // ... 实现细节
    return 0.0; // 简化示例
}

double MathUtils::empirical_inverse_cdf(const std::vector<double>& data, double probability) {
    return quantile(data, probability);
}

double MathUtils::median_rank(const std::vector<double>& data, double value) {
    if (data.empty()) return 0.5;

    std::vector<double> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());

    auto it = std::lower_bound(sorted_data.begin(), sorted_data.end(), value);
    size_t rank = std::distance(sorted_data.begin(), it);

    return (rank + 0.5) / sorted_data.size();
}

double MathUtils::safe_log(double x) {
    return x > 0 ? std::log(x) : 0.0;
}

} // namespace factorlib