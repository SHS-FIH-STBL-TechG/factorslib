// include/utils/trading_time.h
#pragma once
#include <cstdint>

namespace factorlib {

    /**
     * @brief 交易时间判断工具类
     */
    class TradingTime {
    public:
        /**
         * @brief 简化的交易时段判断（A 股日盘）
         * @param ms UNIX 毫秒时间戳
         * @return 是否在 09:30-11:30 与 13:00-15:00 区间内
         */
        static bool in_trading_session_ms(int64_t ms);

        /**
         * @brief 判断是否为集合竞价时段
         * @param ms UNIX 毫秒时间戳
         * @return 是否在集合竞价时段
         */
        static bool in_call_auction_ms(int64_t ms);

        /**
         * @brief 获取下一个交易时段开始时间
         * @param current_ms 当前时间戳
         * @return 下一个交易时段开始时间戳
         */
        static int64_t next_trading_session_start(int64_t current_ms);

        /**
         * @brief 检查是否为有效的交易日期（排除周末和节假日）
         * @param trading_day 交易日（YYYYMMDD格式）
         * @return 是否为有效交易日
         */
        static bool is_valid_trading_day(int trading_day);
    };

} // namespace factorlib