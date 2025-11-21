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

    // === 合表逐笔 ===
    // 合并“逐笔成交 + 逐笔委托”，由 kind 标识本行是成交还是委托
    enum class CombinedKind : uint8_t { Trade = 1, Order = 2 };

    struct CombinedTick {
        CombinedTick() = default;
        std::string instrument_id;   // 证券代码
        int64_t     data_time_ms{0}; // 毫秒时间戳
        uint64_t    main_seq{0};     // 业务序号

        double      price{0.0};
        int         side{0};         // +1 买 / -1 卖（如源是 B/S 或 内/外盘，导入时映射）

        uint64_t    volume{0};       // 成交量(成交) 或 委托数量(委托)

        // 成交专用（成交才有，委托为0）
        uint64_t    bid_no{0};       // 买方订单号
        uint64_t    ask_no{0};       // 卖方订单号

        // 委托专用（委托才有，成交为0）
        uint64_t    order_id{0};     // 委托编号

        CombinedKind kind{CombinedKind::Trade};
        // 允许从 Transaction 隐式转换为 CombinedTick（on_tick(t) 直接生效）
        CombinedTick(const Transaction& t) {
            instrument_id = t.instrument_id;
            data_time_ms  = t.data_time_ms;
            main_seq      = t.main_seq;
            price         = t.price;
            side          = t.side;
            volume        = t.volume;
            bid_no        = t.bid_no;
            ask_no        = t.ask_no;
            order_id      = 0;
            kind          = CombinedKind::Trade;
        }

        // 允许从 Entrust 隐式转换为 CombinedTick（on_tick(e) 直接生效）
        CombinedTick(const Entrust& e) {
            instrument_id = e.instrument_id;
            data_time_ms  = e.data_time_ms;
            main_seq      = e.main_seq;
            price         = e.price;
            side          = e.side;
            volume        = e.volume;
            bid_no        = 0;
            ask_no        = 0;
            order_id      = e.order_id;
            kind          = CombinedKind::Order;
        }

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
