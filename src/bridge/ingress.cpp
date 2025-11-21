// src/bridge/ingress.cpp
#include "bridge/ingress.h"
#include "utils/data_adapter.h"
#include "ifactor.h"
#include "utils/types.h"

#include "../../StrategyPlatform_release/actionType/DataType.h"

namespace {
    std::vector<std::shared_ptr<factorlib::IFactor>> g_factors;
}

namespace factorlib::bridge {

    void set_factors(const std::vector<std::shared_ptr<factorlib::IFactor>>& factors) {
        g_factors = factors;
    }

    void ingest_snapshot_sh(const std::vector<std_SnapshotStockSH>& v) {
        for (const auto& s : v) {
            auto q = factorlib::DataAdapter::from_snapshot_sh(s);
            for (auto& f : g_factors) f->on_quote(q);
        }
    }

    void ingest_snapshot_sz(const std::vector<std_SnapshotStockSZ>& v) {
        for (const auto& s : v) {
            auto q = factorlib::DataAdapter::from_snapshot_sz(s);
            for (auto& f : g_factors) f->on_quote(q);
        }
    }

    void ingest_ont(const std::vector<std_OrdAndExeInfo>& v) {
        for (const auto& x : v) {
            auto tick = factorlib::DataAdapter::to_combined(x);
            for (auto& f : g_factors) {
                f->on_tick(tick);
            }
        }
    }

    void ingest_kline(const std::vector<std_BasicandEnhanceKLine>& v) {
        for (const auto& k : v) {
            auto b = factorlib::DataAdapter::from_kline(k);
            for (auto& f : g_factors) f->on_bar(b);
        }
    }

} // namespace factorlib::bridge