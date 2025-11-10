// src/utils/trading_time.cpp
#include "utils/trading_time.h"
#include <ctime>

namespace factorlib {

    bool TradingTime::in_trading_session_ms(int64_t ms) {
        int64_t sec = (ms/1000) % (24*3600);
        int h = int(sec/3600), m = int((sec%3600)/60);
        if (h<9 || h>15) return false;
        if (h==9 && m<30) return false;
        if (h==11 && m>=30) return false;
        if (h==12) return false;
        if (h==15) return false;
        return true;
    }

    bool TradingTime::in_call_auction_ms(int64_t ms) {
        int64_t sec = (ms/1000) % (24*3600);
        int h = int(sec/3600), m = int((sec%3600)/60), s = int(sec%60);

        // 早盘集合竞价: 09:15-09:25
        if (h == 9 && m >= 15 && m < 25) return true;
        // 尾盘集合竞价: 14:57-15:00
        if (h == 14 && m >= 57) return true;

        return false;
    }

    int64_t TradingTime::next_trading_session_start(int64_t current_ms) {
        // 简化实现：返回下一个交易时段开始时间
        // 实际实现可能需要考虑节假日等复杂情况
        std::time_t time = current_ms / 1000;
        std::tm* tm = std::localtime(&time);

        if (tm->tm_hour < 9 || (tm->tm_hour == 9 && tm->tm_min < 30)) {
            // 当天早盘
            tm->tm_hour = 9;
            tm->tm_min = 30;
            tm->tm_sec = 0;
        } else if (tm->tm_hour < 13) {
            // 当天午盘
            tm->tm_hour = 13;
            tm->tm_min = 0;
            tm->tm_sec = 0;
        } else {
            // 次日早盘
            tm->tm_mday += 1;
            tm->tm_hour = 9;
            tm->tm_min = 30;
            tm->tm_sec = 0;
        }

        return std::mktime(tm) * 1000;
    }

    bool TradingTime::is_valid_trading_day(int trading_day) {
        // 简化实现：实际项目中可能需要接入节假日日历
        // 这里只排除周末
        std::tm tm = {};
        tm.tm_year = trading_day / 10000 - 1900;
        tm.tm_mon = (trading_day % 10000) / 100 - 1;
        tm.tm_mday = trading_day % 100;
        std::mktime(&tm);

        // 周六=6, 周日=0
        return tm.tm_wday != 0 && tm.tm_wday != 6;
    }

} // namespace factorlib