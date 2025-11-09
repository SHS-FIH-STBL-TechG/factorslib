// src/utils/data_adapter.cpp
#include "utils/data_adapter.h"
#include "../../demo_header/DataType.h"

namespace factorlib {

    QuoteDepth DataAdapter::from_snapshot_sh(const SnapshotStockSH& snapshot) {
        QuoteDepth qd;
        qd.instrument_id = security_id_to_string(snapshot.SecurityID);
        qd.data_time_ms = snapshot.TradeTime * 1000LL; // 假设需要转换
        qd.trading_day = snapshot.TradeDate;
        qd.volume = snapshot.TotalVolumeTrade;
        qd.turnover = snapshot.TotalValueTrade;
        qd.bid_price = normalize_price(snapshot.BidPrice01);
        qd.ask_price = normalize_price(snapshot.OfferPrice01);
        return qd;
    }

    double DataAdapter::normalize_price(uint32_t raw_price) {
        return static_cast<double>(raw_price) / 10000.0;
    }

    std::string DataAdapter::security_id_to_string(uint32_t security_id) {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%06u", security_id);
        return std::string(buffer);
    }

} // namespace factorlib