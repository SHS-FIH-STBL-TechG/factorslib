// src/utils/data_adapter.cpp
#include "utils/data_adapter.h"
#include "../../StrategyPlatform_release/actionType/DataType.h"

namespace factorlib {

    QuoteDepth DataAdapter::from_snapshot_sh(const std_SnapshotStockSH& snapshot) {
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


QuoteDepth DataAdapter::from_snapshot_sz(const std_SnapshotStockSZ& snapshot) {
        QuoteDepth qd;
        qd.instrument_id = security_id_to_string(snapshot.SecurityID);
        qd.data_time_ms = snapshot.TradeTime * 1000LL;
        qd.trading_day = snapshot.TradeDate;
        qd.volume = snapshot.TotalVolumeTrade;
        qd.turnover = snapshot.TotalValueTrade;
        qd.bid_price = normalize_price(snapshot.BidPrice01);
        qd.ask_price = normalize_price(snapshot.OfferPrice01);
        return qd;
    }
bool DataAdapter::is_trade(const std_OrdAndExeInfo& x) {
        // 经验规则：TradeQty>0 视为成交；否则视为委托
        return x.TradeQty > 0;
    }


    CombinedTick DataAdapter::to_combined(const std_OrdAndExeInfo& x) {
        CombinedTick tick;
        tick.instrument_id = security_id_to_string(x.SecurityID);
        tick.data_time_ms  = static_cast<int64_t>(x.TradeTime) * 1000LL;
        tick.main_seq      = x.BizIndex;
        tick.price         = normalize_price(x.TradePrice);
        tick.volume        = x.TradeQty;
        tick.side          = (x.BSFlag == 'B') ? 1 : (x.BSFlag == 'S' ? -1 : 0);

        if (is_trade(x)) {
            tick.kind   = CombinedKind::Trade;
            tick.bid_no = x.BuyNo;
            tick.ask_no = x.SellNo;
            tick.order_id = 0;
        } else {
            tick.kind   = CombinedKind::Order;
            tick.bid_no = 0;
            tick.ask_no = 0;
            tick.order_id = (static_cast<uint64_t>(x.ChannelNo) << 32) | x.BuyNo;
        }
        return tick;
    }
Bar DataAdapter::from_kline(const std_BasicandEnhanceKLine& k) {
        Bar b;
        b.instrument_id = security_id_to_string(k.code);
        b.data_time_ms  = static_cast<int64_t>(k.dttm); // 文件定义使用 dttm；单位按源定义，常见为毫秒
        b.open    = k.startPrice;
        b.high    = k.highPrice;
        b.low     = k.lowPrice;
        b.close   = k.endPrice;
        b.volume  = k.tradeVol;
        b.turnover= k.tradeAmt;
        b.interval_ms = 0; // DataType 未提供周期字段，置 0
        return b;
    }
} // namespace factorlib