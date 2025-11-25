// include/utils/processing_axes.h
#pragma once

#include <vector>
#include <mutex>
#include <cstdint>

namespace factorlib {

/**
 * @brief 全局时间频率配置管理。
 *
 * - 默认频率列表为 {1}（逐条计算）。
 * - 通过 set_time_frequencies 在系统入口配置，例如 {1,2,5}。
 * - BaseFactor 及其派生因子从这里读取频率集合并做节奏控制。
 *
 * 该类是一个轻量级的“运行时轴”服务，所有因子共享同一份频率配置，避免各因子重复解析 INI。
 */
class ProcessingAxes {
public:
    static ProcessingAxes& instance();

    /// @brief 设置时间频率列表（<=0 的值会被忽略，单位：毫秒，最终至少保留 1ms）
    void set_time_frequencies(std::vector<int64_t> freqs);

    /// @brief 读取当前时间频率列表（已排序去重，单位：毫秒）
    const std::vector<int64_t>& time_frequencies() const;

private:
    ProcessingAxes();

    /// @brief 清理非法值、排序、去重的通用逻辑，持锁调用
    void normalize_locked(std::vector<int64_t>& freqs) const;

    mutable std::mutex _mutex;
    std::vector<int64_t> _time_freqs;
};

/// @brief 便捷函数：直接设置时间频率列表（供 bridge 层调用）
void set_time_frequencies(const std::vector<int64_t>& freqs);

/// @brief 便捷函数：获取当前时间频率列表
const std::vector<int64_t>& get_time_frequencies();

} // namespace factorlib
