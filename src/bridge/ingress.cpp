// src/bridge/ingress.cpp
#include "factorlib/bridge/ingress.h"
#include "utils/data_adapter.h"
#include "ifactor.h"
#include "utils/types.h"

namespace {
    std::vector<std::shared_ptr<factorlib::IFactor>> g_factors;
}

namespace factorlib::bridge {

    void set_factors(const std::vector<std::shared_ptr<factorlib::IFactor>>& factors) {
        g_factors = factors;
    }

    void ingest_snapshot(const std::vector<SnapshotStockSH>& v) {
        for (const auto& s : v) {
            auto q = factorlib::DataAdapter::from_snapshot_sh(s);
            for (auto& f : g_factors) f->on_quote(q);
        }
    }

    void ingest_ont(const std::vector<OrdAndExeInfo>& v) {
        for (const auto& x : v) {
            if (factorlib::DataAdapter::is_trade(x)) {
                auto t = factorlib::DataAdapter::to_transaction(x);
                for (auto& f : g_factors) f->on_transaction(t);
            } else {
                auto e = factorlib::DataAdapter::to_entrust(x);
                for (auto& f : g_factors) f->on_entrust(e);
            }
        }
    }

    void ingest_kline(const std::vector<BasicandEnhanceKLine>& v) {
        for (const auto& k : v) {
            auto b = factorlib::DataAdapter::from_kline(k);
            for (auto& f : g_factors) f->on_bar(b);
        }
    }

} // namespace factorlib::bridge