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


    Transaction DataAdapter::from_ord_exec(const OrdAndExeInfo& x) {
        Transaction t;
        t.instrument_id = security_id_to_string(x.SecurityID);
        t.data_time_ms  = static_cast<int64_t>(x.TradeTime) * 1000LL;
        t.main_seq      = x.BizIndex;
        t.price         = normalize_price(x.TradePrice);
        t.volume        = x.TradeQty;
        // BSFlag: 66('B') 买，83('S') 卖；无法识别时为0
        t.side          = (x.BSFlag == 'B') ? 1 : (x.BSFlag == 'S' ? -1 : 0);
        t.bid_no        = x.BuyNo;
        t.ask_no        = x.SellNo;
        return t;
    }


    bool DataAdapter::is_trade(const OrdAndExeInfo& x) {
        // 经验规则：TradeQty>0 视为成交；否则视为委托
        return x.TradeQty > 0;
    }


    Transaction DataAdapter::to_transaction(const OrdAndExeInfo& x) {
        return from_ord_exec(x);
    }


    Entrust DataAdapter::to_entrust(const OrdAndExeInfo& x) {
        Entrust e;
        e.instrument_id = security_id_to_string(x.SecurityID);
        e.data_time_ms  = static_cast<int64_t>(x.TradeTime) * 1000LL;
        e.main_seq      = x.BizIndex;
        e.price         = normalize_price(x.TradePrice);
        e.volume        = x.TradeQty;
        // 使用 BSFlag 侧确定方向：买=1 卖=-1
        e.side          = (x.BSFlag == 'B') ? 1 : (x.BSFlag == 'S' ? -1 : 0);
        e.order_id      = static_cast<uint64_t>((static_cast<uint64_t>(x.ChannelNo) << 32) | x.BuyNo);
        return e;
    }


    Bar DataAdapter::from_kline(const BasicandEnhanceKLine& k) {
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