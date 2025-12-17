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
 * 核心功能：将因子原始值转换为标准化的 z-score 杠杆信号
 * 
 * 转换流程：
 * 1. 维护固定长度的滑动窗口（默认 250 个样本，约 1 年交易日）
 * 2. 对窗口内样本按值排序，计算当前值的分位数
 * 3. 使用标准正态分布的逆CDF（分位函数）将分位数映射为 z-score
 * 4. 输出标准正态分布下的 z 值，外部可按需裁剪
 * 
 * 特点：
 * - 无未来信息泄漏：仅使用历史窗口数据
 * - 自动归一化：不同因子映射到统一的正态分布空间
 * - 在线计算：支持实时流式数据处理
 */
class SlidingGaussianLeverage {
public:
    /**
     * @brief 窗口内样本的统计快照
     */
    struct DistributionSnapshot {
        std::size_t sample_count = 0;  ///< 窗口内样本数量
        double min = std::numeric_limits<double>::quiet_NaN();   ///< 最小值
        double max = std::numeric_limits<double>::quiet_NaN();   ///< 最大值
        double mean = std::numeric_limits<double>::quiet_NaN();  ///< 均值
        double std = std::numeric_limits<double>::quiet_NaN();   ///< 标准差
    };

    static constexpr std::size_t kDefaultWindow = 250; ///< 默认窗口大小：250 天（约 1 年交易日）
    
    /**
     * @brief 构造函数
     * @param window 滑动窗口大小（样本数），0 则使用默认值 250
     */
    explicit SlidingGaussianLeverage(std::size_t window = kDefaultWindow)
        : _window_size(window ? window : kDefaultWindow) {}

    /**
     * @brief 将因子值转换为 z-score
     * @param value 因子原始值
     * @return z-score，若输入无效则返回 nullopt
     * 
     * 功能：
     * 1. 将新值加入滑动窗口
     * 2. 更新窗口统计信息
     * 3. 计算当前值在窗口内的排名分位数
     * 4. 通过正态分布逆CDF转换为 z-score
     */
    std::optional<double> transform(double value);

    /**
     * @brief 获取当前窗口的统计信息
     * @return 包含样本数、均值、标准差等的统计快照
     */
    const DistributionSnapshot& stats() const { return _stats; }

private:
    /**
     * @brief 窗口内的单个样本
     */
    struct Sample {
        std::size_t seq;   ///< 序列号，用于同值样本的稳定排序
        double value;      ///< 因子值
    };

    /**
     * @brief 计算当前值的 z-score
     * @return z-score 值，窗口为空则返回 nullopt
     * 
     * 算法：
     * 1. 对窗口内所有样本按值排序（值相同时按序列号排序）
     * 2. 找到最新样本的排名 rank（从 1 开始）
     * 3. 计算分位数 u = (rank - 0.5) / count，范围 [1e-6, 1-1e-6]
     * 4. 使用标准正态分布的分位函数：z = Φ^(-1)(u)
     */
    std::optional<double> compute_leverage();
    
    /**
     * @brief 更新窗口统计信息（均值、标准差、最小/最大值）
     */
    void update_stats();

    std::deque<Sample> _samples;         ///< 滑动窗口样本队列
    std::size_t _next_seq = 0;           ///< 下一个样本的序列号
    std::size_t _window_size = kDefaultWindow;  ///< 窗口大小
    DistributionSnapshot _stats;         ///< 当前统计快照
};

inline std::optional<double> SlidingGaussianLeverage::transform(double value) {
    // 过滤无效输入（NaN、Inf）
    if (!std::isfinite(value)) {
        return std::nullopt;
    }

    // 将新样本加入窗口，分配递增序列号
    _samples.push_back(Sample{_next_seq++, value});
    
    // 维护窗口大小：超出则删除最旧样本
    while (_samples.size() > _window_size) {
        _samples.pop_front();
    }

    // 更新统计信息
    update_stats();
    
    // 计算并返回 z-score
    return compute_leverage();
}

inline void SlidingGaussianLeverage::update_stats() {
    DistributionSnapshot snapshot;
    snapshot.sample_count = _samples.size();
    if (_samples.empty()) {
        _stats = snapshot;
        return;
    }

    // 第一次遍历：计算最小值、最大值、总和
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

    // 第二次遍历：计算方差和标准差
    double var_sum = 0.0;
    for (const auto& s : _samples) {
        const double diff = s.value - snapshot.mean;
        var_sum += diff * diff;
    }
    snapshot.std = std::sqrt(var_sum / count);  // 总体标准差（除以 n）
    _stats = snapshot;
}

inline std::optional<double> SlidingGaussianLeverage::compute_leverage() {
    if (_samples.empty()) {
        return std::nullopt;
    }

    // 复制样本并排序（按值升序，值相同时按序列号升序）
    std::vector<Sample> ranked(_samples.begin(), _samples.end());
    std::sort(ranked.begin(), ranked.end(), [](const Sample& a, const Sample& b) {
        if (std::isnan(a.value) && std::isnan(b.value)) return false;  // NaN 不小于 NaN
        if (std::isnan(a.value)) return false;  // NaN 排在最后
        if (std::isnan(b.value)) return true;   // NaN 排在最后
        if (a.value == b.value) return a.seq < b.seq;  // 值相同按序列号排序（稳定）
        return a.value < b.value;  // 按值升序
    });

    // 找到最新样本（窗口末尾）在排序后的排名
    const std::size_t target_seq = _samples.back().seq;
    std::size_t rank = ranked.size();  // 默认最大排名
    for (std::size_t i = 0; i < ranked.size(); ++i) {
        if (ranked[i].seq == target_seq) {
            rank = i + 1;  // 排名从 1 开始
            break;
        }
    }

    // 计算分位数 u，使用中点调整：(rank - 0.5) / count
    // 避免极端值 0 和 1（会导致 quantile 为 ±∞）
    const double count = static_cast<double>(ranked.size());
    const double u = std::clamp((static_cast<double>(rank) - 0.5) / count,
                                1e-6,
                                1.0 - 1e-6);

    // 使用标准正态分布的分位函数：z = Φ^(-1)(u)
    // 即：若 Z ~ N(0,1)，则 P(Z ≤ z) = u
    static const boost::math::normal normal_dist(0.0, 1.0);
    double z = boost::math::quantile(normal_dist, u);
    return z;
}

} // namespace factorlib::tools
