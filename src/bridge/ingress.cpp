// src/bridge/ingress.cpp
#include "bridge/ingress.h"
#include "utils/data_adapter.h"
#include "ifactor.h"
#include "utils/types.h"
#include "utils/processing_axes.h"

#include "../../StrategyPlatform_release/actionType/DataType.h"

namespace {
    std::vector<std::shared_ptr<factorlib::IFactor>> g_factors;
}

namespace factorlib::bridge {

    void set_factors(const std::vector<std::shared_ptr<factorlib::IFactor>>& factors) {
        g_factors = factors;
    }

    void set_time_frequencies(const std::vector<int>& freqs) {
        // demo 层可以在初始化时调用此函数，把“计算频率轴”注入给所有因子
        factorlib::set_time_frequencies(freqs);
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

    // Overloads for already-converted types (used by tests to avoid dependency on external SDK types)
    void ingest_snapshot_sh(const std::vector<factorlib::QuoteDepth>& v) {
        for (const auto& q : v) {
            for (auto& f : g_factors) f->on_quote(q);
        }
    }

    void ingest_snapshot_sz(const std::vector<factorlib::QuoteDepth>& v) {
        for (const auto& q : v) {
            for (auto& f : g_factors) f->on_quote(q);
        }
    }

    void ingest_ont(const std::vector<factorlib::CombinedTick>& v) {
        for (const auto& t : v) {
            for (auto& f : g_factors) f->on_tick(t);
        }
    }

} // namespace factorlib::bridge
