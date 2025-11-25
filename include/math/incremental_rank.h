// include/utils/math/incremental_rank.h
#pragma once
/**
 * @file incremental_rank.h
 * @brief 增量秩计算器 - 支持滑动窗口内的排序统计和阈值标记
 *
 * 核心功能：
 * - 维护数据的有序视图和时间顺序视图
 * - 支持中位秩计算（经验分布函数）
 * - 基于阈值的0/1标记，支持增量更新
 * - 增量计算标记数据的统计量
 *
 * 应用场景：
 * - 因子计算中的分位数排名
 * - 基于阈值的信号生成
 * - 异常检测和离群值识别
 */

#include <algorithm>
#include <cmath>
#include <deque>
#include <set>
#include <type_traits>
#include <vector>
#include <limits>
#include <unordered_map>

#include "utils/log.h"
#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

/**
 * @class IncrementalRankCalculator
 * @brief 增量秩计算器 - 单变量数据的排序统计和阈值标记
 *
 * 算法复杂度：
 * - push(): O(log n) - 主要开销在multiset插入
 * - update_binary_flags(): 首次O(n)，后续O(k) - k为需要翻转标记的数据量
 * - median_rank(): O(n) - 需要遍历计算位置
 */
template<typename T, typename BadValuePolicy = NoCheckBadValuePolicy>
class IncrementalRankCalculator {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

    using IdType = std::size_t;
    using Pair   = std::pair<T, IdType>; // (数值, 唯一标识)

private:
    // 有序视图：按数值排序，用于快速计算秩和分位数
    std::multiset<Pair> _ordered;

    // 时间窗口：按时间顺序存储，用于维护滑动窗口
    std::deque<Pair> _window;

    // 标记映射：每个数据点的0/1标记（基于当前阈值）
    std::unordered_map<IdType, int> _flags_by_id;

    // 标记数据的增量统计：用于快速计算均值
    double _sum_flagged   = 0.0;      ///< 标记为1的数据值之和
    std::size_t _count_flagged = 0;   ///< 标记为1的数据数量

    // 阈值状态管理
    double _current_threshold = std::numeric_limits<double>::quiet_NaN(); ///< 当前阈值
    bool   _has_threshold     = false; ///< 是否已设置阈值

    // 唯一标识生成器：用于区分相同数值的不同数据点
    IdType _next_id = 0;

public:
    /**
     * @brief 添加新数据点到滑动窗口
     * @param value 数据值
     * @param window_size 窗口大小
     *
     * 处理流程：
     * 1. 坏值检查和处理
     * 2. 生成唯一标识
     * 3. 更新有序视图和时间窗口
     * 4. 如果已设置阈值，立即计算新数据的标记
     * 5. 维护窗口大小，移除过期数据
     *
     * 时间复杂度：O(log n) - multiset插入操作
     */
    void push(T value, std::size_t window_size) {
        // 坏值处理：根据策略决定是否丢弃
        if (!BadValuePolicy::handle(value, "IncrementalRankCalculator::push")) {
            return;  // 策略决定丢弃该数据
        }

        IdType id = _next_id++;
        Pair   p{value, id};

        // 更新数据结构
        _window.push_back(p);          // 时间顺序
        _ordered.insert(p);            // 数值顺序

        // 如果已设置阈值，立即计算新数据的标记
        if (_has_threshold) {
            int flag = (static_cast<double>(value) > _current_threshold) ? 1 : 0;
            _flags_by_id[id] = flag;

            // 更新标记数据的统计量
            if (flag == 1) {
                _sum_flagged += value;
                _count_flagged++;
            }
        }

        // 维护窗口大小：移除最旧数据
        if (_window.size() > window_size) {
            Pair old = _window.front();
            _window.pop_front();

            // 从有序视图中移除
            auto it = _ordered.find(old);
            if (it != _ordered.end())
                _ordered.erase(it);

            // 从标记统计中移除贡献
            auto flag_it = _flags_by_id.find(old.second);
            if (flag_it != _flags_by_id.end()) {
                if (flag_it->second == 1) {
                    _sum_flagged -= old.first;
                    _count_flagged--;
                }
            }

            _flags_by_id.erase(old.second);  // 清理标记映射
        }
    }

    /**
     * @brief 计算指定数值在当前窗口中的中位秩
     * @param value 待查询的数值
     * @return 中位秩，范围[0,1]，表示该数值在排序中的相对位置
     *
     * 中位秩定义：(rank + 0.5) / n
     * 其中rank是小于该数值的数据个数，n是总数据量
     *
     * 应用：将原始数值转换为均匀分布的分位数
     */
    double median_rank(T value) const {
        if (_ordered.empty()) return 0.5;

        // 使用lower_bound找到第一个不小于value的位置
        Pair query{value, std::numeric_limits<IdType>::min()};
        auto it = _ordered.lower_bound(query);
        std::size_t rank = std::distance(_ordered.begin(), it);

        return (rank + 0.5) / static_cast<double>(_ordered.size());
    }

    /// @brief 获取排序后的数据向量（升序）
    std::vector<T> get_sorted_data() const {
        std::vector<T> out;
        out.reserve(_ordered.size());
        for (const auto& p : _ordered) {
            out.push_back(p.first);
        }
        return out;
    }

    /// @brief 获取当前窗口中的数据数量
    std::size_t size() const { return _ordered.size(); }

    /// @brief 清空所有数据和状态
    void clear() {
        _ordered.clear();
        _window.clear();
        _flags_by_id.clear();

        _sum_flagged = 0.0;
        _count_flagged = 0;

        _current_threshold = std::numeric_limits<double>::quiet_NaN();
        _has_threshold = false;
        _next_id = 0;
    }

    /// @brief 检查窗口是否已填满
    bool is_window_full(std::size_t window_size) const {
        return _ordered.size() >= window_size;
    }

    // ================= 阈值标记相关接口 =================

    /**
     * @brief 更新二进制标记的阈值（增量优化）
     * @param new_threshold 新的阈值
     *
     * 算法优化：
     * - 首次调用：全量计算所有数据的标记 O(n)
     * - 后续调用：只更新阈值区间内的数据标记 O(k)，k通常远小于n
     *
     * 标记规则：value > threshold ? 1 : 0
     */
    void update_binary_flags(double new_threshold) {
        if (_window.empty()) {
            // 空窗口：只记录阈值
            _current_threshold = new_threshold;
            _has_threshold = true;
            _flags_by_id.clear();
            _sum_flagged = 0.0;
            _count_flagged = 0;
            return;
        }

        // 首次设置阈值：全量初始化
        if (!_has_threshold) {
            _current_threshold = new_threshold;
            _has_threshold = true;

            _flags_by_id.clear();
            _sum_flagged = 0.0;
            _count_flagged = 0;

            // 遍历所有数据计算初始标记
            for (const auto& p : _ordered) {
                int flag = (static_cast<double>(p.first) > new_threshold) ? 1 : 0;
                _flags_by_id[p.second] = flag;

                if (flag == 1) {
                    _sum_flagged += p.first;
                    _count_flagged++;
                }
            }
            return;
        }

        double old_threshold = _current_threshold;
        if (new_threshold == old_threshold) {
            return; // 阈值未变化
        }

        // 阈值上调：需要将(old, new]区间内的1标记翻转为0
        if (new_threshold > old_threshold) {
            double lo = old_threshold;
            double hi = new_threshold;

            // 找到区间起点：第一个大于lo的位置
            Pair key_lo{static_cast<T>(lo), std::numeric_limits<IdType>::max()};
            auto it = _ordered.upper_bound(key_lo);

            // 遍历区间内的数据，翻转标记
            for (; it != _ordered.end() && static_cast<double>(it->first) <= hi; ++it) {
                IdType id = it->second;

                if (_flags_by_id[id] == 1) {
                    _sum_flagged -= it->first;  // 移除统计贡献
                    _count_flagged--;
                }
                _flags_by_id[id] = 0;  // 翻转为0
            }
        }
        // 阈值下调：需要将(new, old]区间内的0标记翻转为1
        else {
            double lo = new_threshold;
            double hi = old_threshold;

            Pair key_lo{static_cast<T>(lo), std::numeric_limits<IdType>::max()};
            auto it = _ordered.upper_bound(key_lo);

            for (; it != _ordered.end() && static_cast<double>(it->first) <= hi; ++it) {
                IdType id = it->second;

                if (_flags_by_id[id] == 0) {
                    _sum_flagged += it->first;  // 增加统计贡献
                    _count_flagged++;
                }
                _flags_by_id[id] = 1;  // 翻转为1
            }
        }

        _current_threshold = new_threshold;
    }

    /// @brief 获取当前使用的阈值
    double current_threshold() const { return _current_threshold; }

    /**
     * @brief 按时间顺序获取二进制标记序列
     * @return 与_window顺序对应的0/1标记向量
     *
     * 应用：时间序列分析，信号生成
     */
    std::vector<int> get_binary_flags_time_order() const {
        std::vector<int> out;
        out.reserve(_window.size());

        if (!_has_threshold) {
            out.assign(_window.size(), 0);  // 未设置阈值时返回全0
            return out;
        }

        for (const auto& p : _window) {
            auto it = _flags_by_id.find(p.second);
            int flag = (it != _flags_by_id.end()) ? it->second : 0;
            out.push_back(flag);
        }
        return out;
    }

    /**
     * @brief 按数值顺序获取二进制标记序列
     * @return 与_ordered顺序对应的0/1标记向量
     *
     * 应用：分布分析，阈值效果验证
     */
    std::vector<int> get_binary_flags_sorted_order() const {
        std::vector<int> out;
        out.reserve(_ordered.size());

        if (!_has_threshold) {
            out.assign(_ordered.size(), 0);
            return out;
        }

        for (const auto& p : _ordered) {
            auto it = _flags_by_id.find(p.second);
            int flag = (it != _flags_by_id.end()) ? it->second : 0;
            out.push_back(flag);
        }
        return out;
    }

    /**
     * @brief 计算标记为1的数据的均值
     * @return 标记数据的均值，如果没有标记数据返回NaN
     *
     * 应用：条件均值计算，超过阈值数据的统计分析
     */
    double flagged_mean() const {
        if (_count_flagged == 0)
            return std::numeric_limits<double>::quiet_NaN();
        return _sum_flagged / _count_flagged;
    }
};

} // namespace math
} // namespace factorlib