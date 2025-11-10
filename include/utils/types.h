// include/utils/types.h
#pragma once
#include <string>
#include <vector>

namespace factorlib {

    /// L2 行情（最小字段集）
    struct QuoteDepth{
        std::string instrument_id{};
        int64_t data_time_ms{0};
        int trading_day{0};
        uint64_t volume{0};
        double turnover{0.0};
        double bid_price{0.0};
        double ask_price{0.0};
    };

    /// K线（Bar）数据结构
    struct Bar{
        std::string instrument_id{};
        int64_t data_time_ms{0};
        double open{0.0};
        double high{0.0};
        double low{0.0};
        double close{0.0};
        uint64_t volume{0};
        double turnover{0.0};
        int interval_ms{0};
    };

    /// 成交记录（最小字段集）
    struct Transaction{
        std::string instrument_id{};
        int64_t data_time_ms{0};
        uint64_t main_seq{0};
        double price{0.0};
        int side{0};
        uint64_t volume{0};
        uint64_t bid_no{0};
        uint64_t ask_no{0};
    };

    /// 委托记录（最小字段集）
    struct Entrust{
        std::string instrument_id{};
        int64_t data_time_ms{0};
        uint64_t main_seq{0};
        double price{0.0};
        int side{0};
        uint64_t volume{0};
        uint64_t order_id{0};
    };

    /**
     * @brief 单个时间桶的聚合输出
     */
    struct BucketOutputs {
        double amount_sum = 0.0;
        int64_t volume_sum = 0;
        double midprice_last = 0.0;
        std::vector<Transaction> trans;
        std::vector<Entrust> orders;
        int64_t bucket_start_ms = 0;
        int64_t bucket_end_ms = 0;
    };

} // namespace factorlib
