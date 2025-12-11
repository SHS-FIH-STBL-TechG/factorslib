#pragma once

#include "tools/sliding_gaussian_leverage.h"

#include <cstddef>
#include <string>

namespace factorlib::tools {

/**
 * @brief 因子杠杆工具的可配置参数，统一放在此文件，便于后续调优。
 */
struct FactorToolConfig {
    std::string factor_name = "low_freq_return";                  ///< 默认因子名，用于输出目录
    std::size_t default_leverage_window = SlidingGaussianLeverage::kDefaultWindow;
    std::size_t default_topic_capacity = 64'000;
    std::size_t default_lookback_days = 0;                         ///< 0 表示使用全量历史
    double theta_min = 0.0;                                        ///< z-score 空间最小阈值
    double theta_max = 3.0;                                        ///< z-score 空间最大阈值
    double theta_step = 0.1;                                       ///< z-score 网格步长
    double max_leverage = 3.0;                                     ///< 最大杠杆约束
    std::size_t min_trade_days = 50;                               ///< 最少开仓天数
};

/**
 * @brief 获取全局配置（可直接修改本文件中的默认值来调整行为）。
 */
const FactorToolConfig& GetFactorToolConfig();

} // namespace factorlib::tools
