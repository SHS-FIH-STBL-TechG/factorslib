// include/utils/utils.h
#pragma once
#include <cstdint>
#include <string>
#include <vector>

/**
 * @file utils.h
 * @brief 公共数据类型与 N 毫秒桶聚合器的声明。
 *  - 提供 Tick/Transaction/Entrust 的最小数据结构（字段以 demo 为准）。
 *  - 提供 in_trading_session_ms：简化的交易时段判断（A 股日盘）。
 *  - 提供 NmsBucketAggregator：事件（tick/成交/委托）按 N 毫秒对齐到时间桶并累计。
 */

namespace factorlib {

/**
 * @brief 简化的交易时段判断（A 股日盘）
 * @param ms UNIX 毫秒时间戳
 * @return 是否在 09:30-11:30 与 13:00-15:00 区间内
 */
inline bool in_trading_session_ms(int64_t ms){
    int64_t sec = (ms/1000) % (24*3600);
    int h = int(sec/3600), m = int((sec%3600)/60);
    if (h<9 || h>15) return false;
    if (h==9 && m<30) return false;
    if (h==11 && m>=30) return false;
    if (h==12) return false;
    if (h==15) return false;
    return true;
}

// -------------------- 公共数据类型 --------------------

/// L2 行情（最小字段集）
struct QuoteDepth{
    std::string instrument_id{};  ///< 代码，如 "000001.SZ"
    int64_t data_time_ms{0};      ///< 毫秒时间戳
    int trading_day{0};           ///< 交易日（用于跨日增量）
    uint64_t volume{0};           ///< 成交量（累计）
    double turnover{0.0};         ///< 成交额（累计）
    double bid_price{0.0};        ///< 买一价
    double ask_price{0.0};        ///< 卖一价
};

/// K线（Bar）数据结构
struct Bar{
    std::string instrument_id{};  ///< 代码
    int64_t data_time_ms{0};      ///< K线结束时间（毫秒）
    double open{0.0};
    double high{0.0};
    double low{0.0};
    double close{0.0};
    uint64_t volume{0};           ///< 成交量（区间内）
    double turnover{0.0};         ///< 成交额（区间内）
    int interval_ms{0};           ///< 周期（未知则为 0）
};

/// 成交记录（最小字段集）
struct Transaction{
    std::string instrument_id{};
    int64_t data_time_ms{0};
    uint64_t main_seq{0};         ///< 序列号（用于排序）
    double price{0.0};
    int side{0};                  ///< 1 买 -1 卖 0 未知
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
    double amount_sum = 0.0;         ///< 桶内成交额增量之和
    int64_t volume_sum = 0;          ///< 桶内成交量增量之和
    double midprice_last = 0.0;      ///< 桶内最后一笔中价（(bid1+ask1)/2）
    std::vector<Transaction> trans;  ///< 桶内所有成交
    std::vector<Entrust> orders;     ///< 桶内所有委托
    int64_t bucket_start_ms = 0;     ///< 桶开始（含）
    int64_t bucket_end_ms = 0;       ///< 桶结束（不含）
};

/**
 * @brief 事件→N 毫秒时间桶的聚合器（无滑窗；跨桶即产出）
 * 使用方法：
 *   - set_bucket_ms(N)：设置桶宽度（默认 1000ms）；
 *   - on_quote/on_transaction/on_entrust：逐条喂入；
 *   - flush_if_crossed(now_ms, out)：若跨桶则产出上一桶；
 *   - force_flush(out)：强制产出当前桶（一般用于收盘或测试）。
 */
class NmsBucketAggregator {
public:
    explicit NmsBucketAggregator(int64_t bucket_ms=1000):_bucket_ms(bucket_ms){}
    /// 设置时间桶宽度（毫秒）
    void set_bucket_ms(int64_t ms){ _bucket_ms = ms; }
    /// 喂入一条行情（用于增量计算 amount/volume 和更新 midprice）
    void on_quote(const QuoteDepth& q);
    /// 喂入一条成交（仅用于桶切片，不影响 amount/volume）
    void on_transaction(const Transaction& t);
    /// 喂入一条委托（仅用于桶切片）
    void on_entrust(const Entrust& e);
    /**
     * @brief 若 now_ms 已跨越当前桶的尾，则产出上一桶到 out，并开启新桶
     * @return 是否产出了一个桶
     */
    bool flush_if_crossed(int64_t now_ms, BucketOutputs& out);
    /**
     * @brief 强制产出当前桶
     * @return 是否有桶被产出
     */
    bool force_flush(BucketOutputs& out);
    bool ensure_bucket(int64_t ts_ms, BucketOutputs& out);
private:
    int64_t _bucket_ms{1000};
    bool _has_bucket{false};
    BucketOutputs _cur{};
    // 用于增量计算的上一次行情记忆
    bool _has_last_quote{false};
    int _last_trading_day{0};
    uint64_t _last_volume{0};
    double _last_turnover{0.0};
    // 返回值为bool，表示是否跨越了桶边界

    void start_new_bucket(int64_t new_start);
};

} // namespace factorlib
