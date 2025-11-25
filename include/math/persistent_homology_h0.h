#pragma once
/**
 * @file persistent_homology_h0.h
 * @brief 滑动窗口0维持续同调（H0）分析 - 增量实现
 */

#include <set>
#include <deque>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <type_traits>
#include <limits>
#include "math/bad_value_policy.h"

namespace factorlib { namespace math {

template<typename T, typename BadValuePolicy = SkipNaNInfPolicy>
class SlidingWindowPersistenceH0 {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

private:
    std::size_t max_size_;
    std::deque<T> time_window_;
    std::set<T> sorted_points_;
    std::map<T, int> gap_frequency_;
    T total_gap_ = T(0);
    int gap_count_ = 0;
    T eps_max_;
    int total_processed_ = 0;

    void add_gap(T gap) {
        if (gap <= eps_max_ && gap >= T(0)) {
            gap_frequency_[gap]++;
            total_gap_ += gap;
            gap_count_++;
        }
    }

    void remove_gap(T gap) {
        if (gap <= eps_max_ && gap >= T(0)) {
            auto it = gap_frequency_.find(gap);
            if (it != gap_frequency_.end()) {
                total_gap_ -= gap;
                gap_count_--;
                if (--it->second == 0) {
                    gap_frequency_.erase(it);
                }
            }
        }
    }

    T compute_gap(T a, T b) const {
        return T(0.5) * std::abs(b - a);
    }

    void remove_point_from_sorted(T value) {
        auto it = sorted_points_.find(value);
        if (it == sorted_points_.end()) return;

        auto prev = it, next = it;

        T left_gap = T(0), right_gap = T(0);
        bool has_left = false, has_right = false;

        if (it != sorted_points_.begin()) {
            --prev;
            left_gap = compute_gap(*prev, *it);
            has_left = true;
        }

        ++next;
        if (next != sorted_points_.end()) {
            right_gap = compute_gap(*it, *next);
            has_right = true;
        }

        if (has_left) remove_gap(left_gap);
        if (has_right) remove_gap(right_gap);

        if (has_left && has_right) {
            T new_gap = compute_gap(*prev, *next);
            add_gap(new_gap);
        }

        sorted_points_.erase(it);
    }

    void add_point_to_sorted(T value) {
        auto ret = sorted_points_.insert(value);
        if (!ret.second) {
            // 重复点，只添加一次到有序集合中
            return;
        }

        auto it = ret.first;
        auto prev = it, next = it;

        if (it != sorted_points_.begin()) {
            --prev;
            T left_gap = compute_gap(*prev, *it);
            add_gap(left_gap);
        }

        ++next;
        if (next != sorted_points_.end()) {
            T right_gap = compute_gap(*it, *next);
            add_gap(right_gap);
        }

        if (it != sorted_points_.begin() && next != sorted_points_.end()) {
            T old_gap = compute_gap(*prev, *next);
            remove_gap(old_gap);
        }
    }

public:
    explicit SlidingWindowPersistenceH0(std::size_t window_size = 1000,
                                       T eps_max = std::numeric_limits<T>::max())
        : max_size_(window_size), eps_max_(eps_max) {}

    virtual ~SlidingWindowPersistenceH0() = default;

    void reset(std::size_t window_size) {
        max_size_ = window_size;
        clear();
    }

    void clear() {
        time_window_.clear();
        sorted_points_.clear();
        gap_frequency_.clear();
        total_gap_ = T(0);
        gap_count_ = 0;
        total_processed_ = 0;
    }

    bool push(T value) {
        total_processed_++;

        T processed_value = value;
        if (!BadValuePolicy::handle(processed_value, "SlidingWindowPersistenceH0::push")) {
            return false;
        }

        // 添加到时间窗口
        time_window_.push_back(processed_value);

        // 添加到有序集合
        add_point_to_sorted(processed_value);

        // 如果超过窗口大小且窗口大小不为0，移除最旧的点
        if (max_size_ > 0 && time_window_.size() > max_size_) {
            T oldest_value = time_window_.front();
            time_window_.pop_front();
            remove_point_from_sorted(oldest_value);
        }

        return true;
    }

    int push_points(const std::vector<T>& values) {
        int count = 0;
        for (const auto& value : values) {
            if (push(value)) {
                count++;
            }
        }
        return count;
    }

    int push_points(std::initializer_list<T> values) {
        int count = 0;
        for (const auto& value : values) {
            if (push(value)) {
                count++;
            }
        }
        return count;
    }

    T get_mean_persistence() const {
        if (gap_count_ == 0) return T(0);
        return total_gap_ / static_cast<T>(gap_count_);
    }

    bool ready() const {
        // 无限窗口（max_size_ == 0）永远不会ready
        // 有限窗口在达到大小时ready
        return max_size_ > 0 && time_window_.size() >= max_size_;
    }

    std::size_t size() const {
        return time_window_.size();
    }

    std::size_t window_size() const {
        return max_size_;
    }

    int total_processed_points() const {
        return total_processed_;
    }

    int valid_gap_count() const {
        return gap_count_;
    }

    void update_eps_max(T new_eps_max) {
        if (new_eps_max == eps_max_) return;

        eps_max_ = new_eps_max;

        gap_frequency_.clear();
        total_gap_ = T(0);
        gap_count_ = 0;

        if (sorted_points_.size() < 2) return;

        auto it = sorted_points_.begin();
        auto next = it;
        ++next;

        while (next != sorted_points_.end()) {
            T gap = compute_gap(*it, *next);
            add_gap(gap);
            ++it;
            ++next;
        }
    }

    std::vector<T> get_points_time_order() const {
        return std::vector<T>(time_window_.begin(), time_window_.end());
    }

    std::vector<T> get_points_sorted() const {
        return std::vector<T>(sorted_points_.begin(), sorted_points_.end());
    }

    static T compute_static(const std::vector<T>& x, T eps_max) {
        if (x.size() < 2) return T(0);

        std::vector<T> s = x;
        std::sort(s.begin(), s.end());

        std::vector<T> gaps;
        gaps.reserve(s.size() - 1);

        for (size_t i = 1; i < s.size(); ++i) {
            T gap = T(0.5) * std::abs(s[i] - s[i - 1]);
            if (gap <= eps_max) {
                gaps.push_back(gap);
            }
        }

        if (gaps.empty()) return T(0);

        T sum = T(0);
        for (T gap : gaps) sum += gap;
        return sum / static_cast<T>(gaps.size());
    }
};

}} // namespace factorlib::math