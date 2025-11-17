#pragma once
/**
 * @file sliding_statistics.h
 * @brief 固定滑窗的增量统计量（样本数/均值/方差/SSE）。
 */

#include <deque>
#include <cstddef>
#include <type_traits>
#include <limits>
#include <cmath>
#include <utility>

#include "utils/log.h"
#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

template<typename T, typename BadValuePolicy = NoCheckBadValuePolicy>
class SlidingWindowStats {
    static_assert(std::is_arithmetic_v<T>, "T must be arithmetic");

public:
    using value_type = T;

    SlidingWindowStats() = default;

    explicit SlidingWindowStats(std::size_t window_size) {
        reset(window_size);
    }

    /// 重置窗口大小并清空状态
    void reset(std::size_t window_size) {
        max_size_ = window_size;
        window_.clear();
        n_ = 0;
        sum_ = 0;
        sumsq_ = 0;
    }

    /// 清空但保持窗口大小
    void clear() {
        reset(max_size_);
    }

    std::size_t window_size() const { return max_size_; }
    std::size_t size()        const { return n_; }

    /// 追加一条样本；超过窗口时自动弹出最旧的一条
    void push(T v) {
        // 统一入口：先按策略处理 NaN/Inf 等异常值
        if (!BadValuePolicy::handle(v, "SlidingWindowStats::push")) {
            // 策略要求丢弃该样本
            return;
        }

        if (max_size_ == 0) {
            // 兜底：0 视为“无限窗口”，只累积不丢弃
            window_.push_back(v);
            ++n_;
            sum_   += static_cast<long double>(v);
            sumsq_ += static_cast<long double>(v) * static_cast<long double>(v);
            return;
        }

        if (n_ == max_size_) {
            T old = window_.front();
            window_.pop_front();
            sum_   -= static_cast<long double>(old);
            sumsq_ -= static_cast<long double>(old) * static_cast<long double>(old);
            --n_;
        }

        window_.push_back(v);
        ++n_;
        sum_   += static_cast<long double>(v);
        sumsq_ += static_cast<long double>(v) * static_cast<long double>(v);
    }

    /// 是否已经“满窗”
    bool ready() const {
        return max_size_ > 0 && n_ >= max_size_;
    }

    /// 是否有足够样本计算样本方差（n>=2）
    bool has_variance() const {
        return n_ >= 2;
    }

    /// 当前窗口内的样本均值
    double mean() const {
        if (n_ == 0) return std::numeric_limits<double>::quiet_NaN();
        return static_cast<double>(sum_ / static_cast<long double>(n_));
    }

    /// 当前窗口的样本方差（除以 n-1）
    double variance_sample() const {
        if (n_ < 2) return std::numeric_limits<double>::quiet_NaN();
        long double sse = sumsq_ - (sum_ * sum_) / static_cast<long double>(n_);
        if (sse < 0) sse = 0;
        return static_cast<double>(sse / static_cast<long double>(n_ - 1));
    }

    /// 当前窗口相对均值的 SSE
    double sse() const {
        if (n_ == 0) return 0.0;
        long double sse = sumsq_ - (sum_ * sum_) / static_cast<long double>(n_);
        if (sse < 0) sse = 0;
        return static_cast<double>(sse);
    }

private:
    std::size_t max_size_{0};
    std::size_t n_{0};
    std::deque<T> window_;
    long double sum_{0};
    long double sumsq_{0};
};

    // ===================== 加权滑动统计（value, weight） =====================

template<typename T, typename W = double, typename BadValuePolicy = SkipNaNInfPolicy>
class WeightedSlidingWindowStats {
    static_assert(std::is_arithmetic_v<T>, "T must be arithmetic");
    static_assert(std::is_arithmetic_v<W>, "W must be arithmetic");

public:
    WeightedSlidingWindowStats() = default;

    explicit WeightedSlidingWindowStats(std::size_t max_size) {
        reset(max_size);
    }

    /// 重置窗口大小并清空历史
    void reset(std::size_t max_size) {
        max_size_ = max_size;
        values_.clear();
        weights_.clear();
        sum_w_   = 0.0L;
        sum_wr_  = 0.0L;
        sum_wr2_ = 0.0L;
    }

    /// 推入一个新样本 (value, weight)
    void push(T value, W weight) {
        // 1) 先按策略处理 value（可能会被置零或被丢弃）
        if (!BadValuePolicy::handle(value, "WeightedSlidingWindowStats::push")) {
            return;  // 丢弃这一条样本
        }

        // 2) 检查权重是否合法：必须是有限值且 > 0
        if (!is_finite_numeric(weight) || !(weight > static_cast<W>(0))) {
            LOG_WARN("WeightedSlidingWindowStats::push: invalid weight {}, drop sample", weight);
            return;
        }

        if (max_size_ == 0) return;

        values_.push_back(value);
        weights_.push_back(weight);

        const long double v = static_cast<long double>(value);
        const long double w = static_cast<long double>(weight);

        sum_w_   += w;
        sum_wr_  += w * v;
        sum_wr2_ += w * v * v;

        // 控制窗口长度
        while (values_.size() > max_size_) {
            T v_old = values_.front(); values_.pop_front();
            W w_old = weights_.front(); weights_.pop_front();

            const long double v0 = static_cast<long double>(v_old);
            const long double w0 = static_cast<long double>(w_old);

            sum_w_   -= w0;
            sum_wr_  -= w0 * v0;
            sum_wr2_ -= w0 * v0 * v0;
        }

        if (sum_w_ < 0.0L) sum_w_ = 0.0L;
    }

    std::size_t size() const { return values_.size(); }

    double sum_w() const {
        return static_cast<double>(sum_w_);
    }

    /// 计算加权均值和标准差；成功返回 true，失败返回 false（样本不足）
    bool mean_var(double& mu, double& sigma) const {
        if (values_.size() < 2 || !(sum_w_ > 0.0L)) {
            mu    = 0.0;
            sigma = 0.0;
            return false;
        }

        const long double w  = sum_w_;
        const long double m1 = sum_wr_  / w;
        const long double m2 = sum_wr2_ / w;
        long double var = m2 - m1 * m1;
        if (var < 0.0L) var = 0.0L;

        mu    = static_cast<double>(m1);
        sigma = std::sqrt(static_cast<double>(var));
        return true;
    }

private:
    std::size_t max_size_{0};
    std::deque<T> values_;
    std::deque<W> weights_;
    long double sum_w_{0.0L};
    long double sum_wr_{0.0L};
    long double sum_wr2_{0.0L};
};


}} // namespace factorlib::math
