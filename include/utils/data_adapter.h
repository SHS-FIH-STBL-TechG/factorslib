// include/utils/data_adapter.h
#pragma once
#include "utils/utils.h"
#include <string>

struct SnapshotStockSH;
struct std_SnapshotStockSZ;
struct OrdAndExeInfo;
struct BasicandEnhanceKLine;

namespace factorlib {

    class DataAdapter {
    public:
        // 将原始快照数据转换为标准 QuoteDepth
        static QuoteDepth from_snapshot_sh(const SnapshotStockSH& snapshot);
        static QuoteDepth from_snapshot_sz(const std_SnapshotStockSZ& snapshot);

        // 将成交数据转换为标准 Transaction
        static Transaction from_ord_exec(const OrdAndExeInfo& ord_exec);

        // 逐笔委托/成交的拆分与转换
        static bool is_trade(const OrdAndExeInfo& x);
        static Transaction to_transaction(const OrdAndExeInfo& x);
        static Entrust     to_entrust(const OrdAndExeInfo& x);

        // K线转换
        static Bar from_kline(const BasicandEnhanceKLine& k);


        // 工具函数：价格转换（假设原始价格需要除以10000）
        static double normalize_price(uint32_t raw_price);
        static std::string security_id_to_string(uint32_t security_id);
    };

} // namespace factorlib
