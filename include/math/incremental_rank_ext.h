// include/utils/math/incremental_rank_with_aux.h
#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <set>
#include <type_traits>
#include <vector>
#include <limits>
#include <unordered_map>

#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

/**
 * @brief 带 AUX 的增量秩计算器
 *
 * - 逻辑完全仿照 IncrementalRankCalculator：
 *   - multiset<(value, id)> 保持排序视图
 *   - deque<(value, id)>    保持时间顺序窗口
 *   - update_binary_flags(threshold) 增量维护 0/1 flag
 *
 * - 额外维护：
 *   - 与时间窗口对齐的 AUX 序列 aux[i]
 *   - 基于 flag(id)=1 的：
 *       * 主序列增量均值 flagged_mean()
 *       * AUX 序列增量均值 aux_flagged_mean()
 */
template<typename T,
         typename AuxT = double,
         typename BadValuePolicy = NoCheckBadValuePolicy>
class IncrementalRankWithAux {
    static_assert(std::is_arithmetic_v<T>,    "T must be an arithmetic type");
    static_assert(std::is_arithmetic_v<AuxT>, "AuxT must be an arithmetic type");

public:
    using IdType = std::size_t;
    using Pair   = std::pair<T, IdType>; // (value, id)

private:
    // ---- 主序列视图 ----
    std::multiset<Pair> _ordered;         // 按 (value, id) 排序
    std::deque<Pair>    _window;          // 按时间顺序

    // id -> flag(0/1)
    std::unordered_map<IdType, int> _flags_by_id;

    // flag=1 的主序列均值增量统计
    double      _sum_flagged   = 0.0;
    std::size_t _count_flagged = 0;

    // 当前阈值
    double _current_threshold  = std::numeric_limits<double>::quiet_NaN();
    bool   _has_threshold      = false;

    // id 递增
    IdType _next_id = 0;

    // ---- AUX 增量维护 ----
    std::deque<AuxT> _aux_window;                 // 与 _window 对齐（时间顺序）
    std::unordered_map<IdType, AuxT> _aux_by_id;  // id -> aux

    double      _aux_sum_flag   = 0.0;  // 仅统计 flag=1 的 AUX
    std::size_t _aux_count_flag = 0;

public:
    IncrementalRankWithAux() = default;

    // ============================================================
    // push：增量 O(log N)（插入 + 窗口滑动），无全量遍历
    // ============================================================
    void push(T value, AuxT aux_value, std::size_t window_size) {
        // 坏值策略（如果你需要的话）
        if (!BadValuePolicy::handle(value, "IncrementalRankWithAux::push")) {
            return; // 丢弃该条记录
        }

        IdType id = _next_id++;
        Pair   p{ value, id };

        // 1) 主序列插入
        _window.push_back(p);
        _ordered.insert(p);

        // 2) AUX 插入
        _aux_window.push_back(aux_value);
        _aux_by_id[id] = aux_value;

        // 3) 若已经有阈值，则为新点打 flag，并同步增量更新两种均值
        if (_has_threshold) {
            int f = (static_cast<double>(value) > _current_threshold) ? 1 : 0;
            _flags_by_id[id] = f;

            if (f == 1) {
                _sum_flagged   += value;
                _count_flagged++;

                _aux_sum_flag  += aux_value;
                _aux_count_flag++;
            }
        }

        // 4) 窗口超长 → 删除最旧点（增量）
        if (_window.size() > window_size) {
            // 主序列最旧点
            Pair old = _window.front();
            _window.pop_front();

            // AUX 最旧点（时间顺序对齐）
            AuxT old_aux = _aux_window.front();
            _aux_window.pop_front();

            // ordered 中删除旧点（O(log N)）
            auto it = _ordered.find(old);
            if (it != _ordered.end()) {
                _ordered.erase(it);
            }

            // 若旧点 flag=1，则从增量均值中扣除
            auto itf = _flags_by_id.find(old.second);
            if (itf != _flags_by_id.end()) {
                if (itf->second == 1) {
                    _sum_flagged   -= old.first;
                    _count_flagged--;

                    _aux_sum_flag  -= old_aux;
                    _aux_count_flag--;
                }
            }

            _flags_by_id.erase(old.second);
            _aux_by_id.erase(old.second);
        }
    }

    // ============================================================
    // median_rank：查询接口，内部需要一次遍历 ordered（O(log N) 查下标，O(N) distance）
    // ============================================================
    double median_rank(T value) const {
        if (_ordered.empty()) return 0.5;

        Pair query{ value, std::numeric_limits<IdType>::min() };
        auto it = _ordered.lower_bound(query);
        std::size_t rank = std::distance(_ordered.begin(), it);
        return (rank + 0.5) / static_cast<double>(_ordered.size());
    }

    // ============================================================
    // get_sorted_data：查询接口，O(N) 拷贝
    // ============================================================
    std::vector<T> get_sorted_data() const {
        std::vector<T> out;
        out.reserve(_ordered.size());
        for (const auto& p : _ordered) {
            out.push_back(p.first);
        }
        return out;
    }

    // ============================================================
    // 基本信息
    // ============================================================
    std::size_t size() const { return _ordered.size(); }

    bool is_window_full(std::size_t window_size) const {
        return _ordered.size() >= window_size;
    }

    void clear() {
        _ordered.clear();
        _window.clear();
        _flags_by_id.clear();

        _sum_flagged   = 0.0;
        _count_flagged = 0;

        _current_threshold = std::numeric_limits<double>::quiet_NaN();
        _has_threshold     = false;
        _next_id           = 0;

        _aux_window.clear();
        _aux_by_id.clear();
        _aux_sum_flag   = 0.0;
        _aux_count_flag = 0;
    }

    // ============================================================
    // update_binary_flags：**关键增量步骤**
    // ============================================================
    void update_binary_flags(double new_threshold) {
        if (_window.empty()) {
            // 空窗口：只记录阈值和清空统计
            _current_threshold = new_threshold;
            _has_threshold     = true;

            _flags_by_id.clear();
            _sum_flagged   = 0.0;
            _count_flagged = 0;

            _aux_sum_flag   = 0.0;
            _aux_count_flag = 0;
            return;
        }

        // 1) 第一次设置阈值：**这里是一次性全量 O(N)**（必须汇报）
        if (!_has_threshold) {
            _current_threshold = new_threshold;
            _has_threshold     = true;

            _flags_by_id.clear();
            _sum_flagged   = 0.0;
            _count_flagged = 0;

            _aux_sum_flag   = 0.0;
            _aux_count_flag = 0;

            // 扫一遍当前窗口（时间顺序），初始化 flag 和两种均值
            for (const auto& p : _window) {
                T      v  = p.first;
                IdType id = p.second;
                int f = (static_cast<double>(v) > new_threshold) ? 1 : 0;
                _flags_by_id[id] = f;

                if (f == 1) {
                    _sum_flagged   += v;
                    _count_flagged++;

                    auto it_aux = _aux_by_id.find(id);
                    if (it_aux != _aux_by_id.end()) {
                        _aux_sum_flag  += it_aux->second;
                        _aux_count_flag++;
                    }
                }
            }
            return;
        }

        // 2) 后续更新：增量，只扫描 (old_th, new_th] 区间内的元素
        double old_th = _current_threshold;
        if (new_threshold == old_th) {
            return; // 阈值没变，无需更新
        }

        // 阈值上调：old < new → 区间( old, new ] 中原本 flag=1 的元素变成 0
        if (new_threshold > old_th) {
            double lo = old_th;
            double hi = new_threshold;

            Pair key_lo{ static_cast<T>(lo), std::numeric_limits<IdType>::max() };
            auto it = _ordered.upper_bound(key_lo);
            for (; it != _ordered.end() && static_cast<double>(it->first) <= hi; ++it) {
                T      v  = it->first;
                IdType id = it->second;

                auto itf = _flags_by_id.find(id);
                if (itf == _flags_by_id.end()) continue;
                if (itf->second == 1) {
                    // 1 -> 0
                    _sum_flagged   -= v;
                    _count_flagged--;

                    auto it_aux = _aux_by_id.find(id);
                    if (it_aux != _aux_by_id.end()) {
                        _aux_sum_flag  -= it_aux->second;
                        _aux_count_flag--;
                    }

                    itf->second = 0;
                }
            }
        }
        // 阈值下调：new < old → 区间( new, old ] 中原本 flag=0 的元素变成 1
        else {
            double lo = new_threshold;
            double hi = old_th;

            Pair key_lo{ static_cast<T>(lo), std::numeric_limits<IdType>::max() };
            auto it = _ordered.upper_bound(key_lo);
            for (; it != _ordered.end() && static_cast<double>(it->first) <= hi; ++it) {
                T      v  = it->first;
                IdType id = it->second;

                auto itf = _flags_by_id.find(id);
                if (itf == _flags_by_id.end()) continue;
                if (itf->second == 0) {
                    // 0 -> 1
                    _sum_flagged   += v;
                    _count_flagged++;

                    auto it_aux = _aux_by_id.find(id);
                    if (it_aux != _aux_by_id.end()) {
                        _aux_sum_flag  += it_aux->second;
                        _aux_count_flag++;
                    }

                    itf->second = 1;
                }
            }
        }

        _current_threshold = new_threshold;
    }

    // ============================================================
    // flag / 均值 查询接口（O(1)）
    // ============================================================
    double current_threshold() const {
        return _current_threshold;
    }

    double flagged_mean() const {
        if (_count_flagged == 0)
            return std::numeric_limits<double>::quiet_NaN();
        return _sum_flagged / _count_flagged;
    }

    double aux_flagged_mean() const {
        if (_aux_count_flag == 0)
            return std::numeric_limits<double>::quiet_NaN();
        return _aux_sum_flag / _aux_count_flag;
    }

    // ============================================================
    // 按时间顺序的 flag 向量（查询接口 O(N)）
    // ============================================================
    std::vector<int> get_binary_flags_time_order() const {
        std::vector<int> out;
        out.reserve(_window.size());

        if (!_has_threshold) {
            out.assign(_window.size(), 0);
            return out;
        }

        for (const auto& p : _window) {
            IdType id = p.second;
            auto it = _flags_by_id.find(id);
            int f = (it != _flags_by_id.end()) ? it->second : 0;
            out.push_back(f);
        }
        return out;
    }

    // ============================================================
    // 按数值排序的 flag 向量（查询接口 O(N)）
    // ============================================================
    std::vector<int> get_binary_flags_sorted_order() const {
        std::vector<int> out;
        out.reserve(_ordered.size());

        if (!_has_threshold) {
            out.assign(_ordered.size(), 0);
            return out;
        }

        for (const auto& p : _ordered) {
            IdType id = p.second;
            auto it = _flags_by_id.find(id);
            int f = (it != _flags_by_id.end()) ? it->second : 0;
            out.push_back(f);
        }
        return out;
    }
};

} // namespace math
} // namespace factorlib
