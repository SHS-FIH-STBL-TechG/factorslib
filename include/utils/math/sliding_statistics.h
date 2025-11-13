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

namespace factorlib { namespace math {

template<typename T>
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

}} // namespace factorlib::math
