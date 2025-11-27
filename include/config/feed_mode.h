#pragma once
/**
 * @file feed_mode.h
 * @brief 全局“喂数路径”配置：逐条事件 / 聚合（NmsBucketAggregator）。
 *
 * 设计要点：
 *   1) 这是“运行配置”，应与 DataBus 解耦，放在 utils/config 层最合适。
 *   2) 支持编译期默认值（宏），也支持运行时在具体因子里覆写。
 *
 * 使用方式：
 *   - 编译期：add_definitions(-DFACTORLIB_FEEDMODE_USE_AGGREGATOR_DEFAULT=1) 切换为聚合默认
 *   - 运行时：在因子 Config 中显式设置 feed_mode 覆盖全局默认
 */

#ifndef FACTORLIB_FEEDMODE_USE_AGGREGATOR_DEFAULT
#define FACTORLIB_FEEDMODE_USE_AGGREGATOR_DEFAULT 0   // 0=逐条事件；1=聚合
#endif

#include <cstdint>

namespace factorlib::config {

    enum class FeedMode : uint8_t {
        EventDriven = 0,  ///< 逐条事件（Quote/Entrust/Transaction 来一条，计算一条）
        Aggregated  = 1   ///< 聚合（NmsBucketAggregator 出一条聚合样本）
    };

    /** @brief 返回编译期默认喂数模式（全局默认）。个别因子可在运行时覆盖。 */
    inline FeedMode global_default_feed_mode() {
#if FACTORLIB_FEEDMODE_USE_AGGREGATOR_DEFAULT
        return FeedMode::Aggregated;
#else
        return FeedMode::EventDriven;
#endif
    }

} // namespace factorlib::config
