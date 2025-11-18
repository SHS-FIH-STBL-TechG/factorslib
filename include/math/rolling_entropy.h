#pragma once
/**
 * @file rolling_entropy.h
 * @brief 固定滑窗的直方图香农熵 H = -sum p_i log p_i（增量 O(1)）。
 *
 * 设计：固定分箱边界，维护每个箱子的计数，在滑窗进/出时更新计数和样本总数。
 */
#include <vector>
#include <deque>
#include <cmath>
#include <limits>
#include <algorithm>
#include <type_traits>

#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

template<typename T=float, typename BadPolicy=NoCheckBadValuePolicy>
class RollingHistogramEntropy {
public:
    RollingHistogramEntropy(std::size_t W, std::vector<double> bin_edges)
        : W_(W), edges_(std::move(bin_edges)) {
        if (edges_.size() < 2) throw std::invalid_argument("edges must have >=2");
        counts_.assign(edges_.size() - 1, 0);
        buf_.assign(W_, std::numeric_limits<T>::quiet_NaN());
    }

    bool push(T x) {
        if constexpr (std::is_floating_point_v<T>) {
            if (!is_finite_numeric(x)) {
                if (!BadPolicy::handle(x, "RollingHistogramEntropy::push")) {
                    return false;
                }
            }
        }
        if (size_ < W_) {
            buf_[tail_] = x;
            if (is_valid(x)) {
                ++valid_;
                counts_[bin_of(x)]++;
            }
            tail_ = (tail_ + 1) % W_;
            ++size_;
            return true;
        }

        // 弹出最老
        T x_old = buf_[head_];
        if (is_valid(x_old)) {
            --valid_;
            counts_[bin_of(x_old)]--;
        }
        // 写入新
        buf_[head_] = x;
        head_ = (head_ + 1) % W_;
        tail_ = (tail_ + 1) % W_;
        if (is_valid(x)) {
            ++valid_;
            counts_[bin_of(x)]++;
        }
        return true;
    }

    bool ready() const { return size_ >= W_; }

    double entropy() const {
        if (valid_ == 0) return std::numeric_limits<double>::quiet_NaN();
        double H = 0.0;
        for (auto c : counts_) {
            if (c <= 0) continue;
            double p = static_cast<double>(c) / static_cast<double>(valid_);
            H -= p * std::log(p);
        }
        return H;
    }

private:
    static bool is_valid(T x) {
        if constexpr (std::is_floating_point_v<T>) return is_finite_numeric(x);
        return true;
    }
    std::size_t bin_of(T x) const {
        // 左闭右开 [e[i], e[i+1])
        auto it = std::upper_bound(edges_.begin(), edges_.end(), static_cast<double>(x));
        std::size_t idx = (it == edges_.begin()) ? 0 : static_cast<std::size_t>(it - edges_.begin() - 1);
        if (idx >= counts_.size()) idx = counts_.size()-1; // 落在最右边界上
        return idx;
    }

private:
    std::size_t W_;
    std::vector<double> edges_;
    std::vector<int> counts_;
    std::vector<T> buf_;
    std::size_t head_{0}, tail_{0}, size_{0};
    std::size_t valid_{0};
};

}} // namespace factorlib::math
