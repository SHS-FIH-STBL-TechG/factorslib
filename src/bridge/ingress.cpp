// src/bridge/ingress.cpp
#include "bridge/ingress.h"
#include "utils/data_adapter.h"
#include "ifactor.h"
#include "utils/types.h"
#include "utils/processing_axes.h"

#include <algorithm>
#ifdef FACTORLIB_ENABLE_PARALLEL_INGRESS
#include <execution>
#endif

#include "../../StrategyPlatform_release/actionType/DataType.h"

namespace {
    std::vector<std::shared_ptr<factorlib::IFactor>> g_factors;

    template <typename Fn>
    void dispatch_to_factors(Fn&& fn) {
#ifdef FACTORLIB_ENABLE_PARALLEL_INGRESS
        std::for_each(std::execution::par, g_factors.begin(), g_factors.end(),
                      [&](const std::shared_ptr<factorlib::IFactor>& f) {
                          if (f) fn(*f);
                      });
#else
        for (auto& f : g_factors) {
            if (f) fn(*f);
        }
#endif
    }
}

namespace factorlib::bridge {

    void set_factors(const std::vector<std::shared_ptr<factorlib::IFactor>>& factors) {
        g_factors = factors;
    }

    void set_time_frequencies(const std::vector<int64_t>& freqs) {
        // demo 层可以在初始化时调用此函数，把“计算频率轴”注入给所有因子
        factorlib::set_time_frequencies(freqs);
    }

    void ingest_snapshot_sh(const std::vector<std_SnapshotStockSH>& v) {
        for (const auto& s : v) {
            auto q = factorlib::DataAdapter::from_snapshot_sh(s);
            dispatch_to_factors([&](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_snapshot_sz(const std::vector<std_SnapshotStockSZ>& v) {
        for (const auto& s : v) {
            auto q = factorlib::DataAdapter::from_snapshot_sz(s);
            dispatch_to_factors([&](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_ont(const std::vector<std_OrdAndExeInfo>& v) {
        for (const auto& x : v) {
            auto tick = factorlib::DataAdapter::to_combined(x);
            dispatch_to_factors([&](factorlib::IFactor& f){ f.on_tick(tick); });
        }
    }

    void ingest_kline(const std::vector<std_BasicandEnhanceKLine>& v) {
        for (const auto& k : v) {
            auto b = factorlib::DataAdapter::from_kline(k);
            dispatch_to_factors([&](factorlib::IFactor& f){ f.on_bar(b); });
        }
    }

    // Overloads for already-converted types (used by tests to avoid dependency on external SDK types)
    void ingest_snapshot_sh(const std::vector<factorlib::QuoteDepth>& v) {
        for (const auto& q : v) {
            dispatch_to_factors([&](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_snapshot_sz(const std::vector<factorlib::QuoteDepth>& v) {
        for (const auto& q : v) {
            dispatch_to_factors([&](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_ont(const std::vector<factorlib::CombinedTick>& v) {
        for (const auto& t : v) {
            dispatch_to_factors([&](factorlib::IFactor& f){ f.on_tick(t); });
        }
    }

} // namespace factorlib::bridge
