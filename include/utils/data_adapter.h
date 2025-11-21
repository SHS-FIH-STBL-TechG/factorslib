// include/utils/data_adapter.h
#pragma once
#include <string>
#include "utils/trading_time.h"
#include "utils/types.h"

struct std_SnapshotStockSH;
struct std_SnapshotStockSZ;
struct std_OrdAndExeInfo;
struct std_BasicandEnhanceKLine;

namespace factorlib {

    class DataAdapter {
    public:
        // 将原始快照数据转换为标准 QuoteDepth
        static QuoteDepth from_snapshot_sh(const std_SnapshotStockSH& snapshot);
        static QuoteDepth from_snapshot_sz(const std_SnapshotStockSZ& snapshot);

        // // 将成交数据转换为标准 Transaction
        // static Transaction from_ord_exec(const OrdAndExeInfo& ord_exec);

        // 逐笔委托/成交的拆分与转换
        static bool is_trade(const std_OrdAndExeInfo& x);
        // ONT 直接转换为 CombinedTick，避免额外中间对象
        static CombinedTick to_combined(const std_OrdAndExeInfo& x);

        // K线转换
        static Bar from_kline(const std_BasicandEnhanceKLine& k);


        // 工具函数：价格转换（假设原始价格需要除以10000）
        static double normalize_price(uint32_t raw_price);
        static std::string security_id_to_string(uint32_t security_id);
    };

} // namespace factorlib
