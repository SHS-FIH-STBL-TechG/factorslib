// tests/utils/data_gen.h
#pragma once
#include <vector>
#include "core/types.h"                    // 只需要数据类型
#include "utils/trading_time.h"             // 需要交易时间函数
#include "utils/math_utils.h"               // 需要数学工具
#include "utils/nms_bucket_aggregator.h"    // 需要时间桶聚合器

/**
 * @file data_gen.h
 * @brief 测试数据生成工具：构造一段可预期的行情/成交/委托序列，便于精确断言。
 */

namespace factorlib { namespace testutil {

/// 工具：把 (H,M,S,ms) 转成“当天毫秒偏移”（仅测试使用）
inline int64_t hms_ms(int H,int M,int S,int ms=0){
    return ((H*3600LL + M*60LL + S)*1000LL + ms);
}

/// 一段时间内的合成序列
struct Series {
    std::vector<QuoteDepth> quotes;  ///< 行情序列（用于增量与中价）
    std::vector<Transaction> trans;  ///< 成交序列（用于切片）
    std::vector<Entrust> orders;     ///< 委托序列（用于切片）
};

/**
 * @brief 构造一个“2 个时间桶”的基本样例
 *  - 桶宽 bucket_ms，起点 start_ms；
 *  - 第 1 桶：累计成交量/额增加 400、40000；
 *  - 第 2 桶：累计成交量/额增加 600、60000；
 *  - 分别插入成交/委托用于切片断言。
 */
inline Series make_series_basic(const std::string& code, int64_t start_ms, int64_t bucket_ms){
    Series s;
    QuoteDepth q{};
    q.instrument_id=code; q.trading_day=20250101; q.bid_price=10.0; q.ask_price=10.2;
    q.data_time_ms=start_ms; q.volume=1000; q.turnover=100000.0; s.quotes.push_back(q);
    q.data_time_ms=start_ms+400; q.volume=1200; q.turnover=120000.0; s.quotes.push_back(q); // +200/+20000
    q.data_time_ms=start_ms+900; q.volume=1400; q.turnover=140000.0; s.quotes.push_back(q); // +200/+20000
    q.data_time_ms=start_ms+bucket_ms+100; q.volume=1600; q.turnover=160000.0; s.quotes.push_back(q); // +200/+20000
    q.data_time_ms=start_ms+bucket_ms+800; q.volume=2000; q.turnover=200000.0; s.quotes.push_back(q); // +400/+40000

    Transaction t{}; t.instrument_id=code;
    t.data_time_ms=start_ms+100; t.main_seq=1; t.price=10.1; t.volume=50; s.trans.push_back(t);
    t.data_time_ms=start_ms+700; t.main_seq=2; t.price=10.1; t.volume=70; s.trans.push_back(t);
    t.data_time_ms=start_ms+bucket_ms+300; t.main_seq=3; t.price=10.2; t.volume=80; s.trans.push_back(t);

    Entrust e{}; e.instrument_id=code;
    e.data_time_ms=start_ms+500; e.main_seq=5; e.price=10.15; e.volume=100; s.orders.push_back(e);
    e.data_time_ms=start_ms+bucket_ms+600; e.main_seq=6; e.price=10.18; e.volume=120; s.orders.push_back(e);

    return s;
}

}} // namespace
