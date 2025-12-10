// src/bridge/ingress.cpp
#include "bridge/ingress.h"
#include "utils/data_adapter.h"
#include "instrumentation/trace_helper.h"
#include "core/ifactor.h"
#include "core/types.h"

#include <algorithm>
#include <functional>
#include <memory>
#ifdef FACTORLIB_ENABLE_PARALLEL_INGRESS
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#endif

#include "../../StrategyPlatform_release/actionType/DataType.h"

namespace {
    std::vector<std::shared_ptr<factorlib::IFactor>> g_factors;
#ifdef FACTORLIB_ENABLE_PARALLEL_INGRESS
    struct FactorWorker {
        explicit FactorWorker(std::shared_ptr<factorlib::IFactor> f)
            : factor(std::move(f)), thread(&FactorWorker::run, this) {}

        ~FactorWorker() { shutdown(); }

        void enqueue(std::function<void(factorlib::IFactor&)> task) {
            {
                std::lock_guard<std::mutex> lk(mtx);
                tasks.emplace_back(std::move(task));
            }
            cv.notify_one();
        }

        void shutdown() {
            if (joined) return;
            {
                std::lock_guard<std::mutex> lk(mtx);
                stop = true;
            }
            cv.notify_one();
            if (thread.joinable()) thread.join();
            joined = true;
        }

        std::shared_ptr<factorlib::IFactor> factor;
        std::thread thread;
        std::mutex mtx;
        std::condition_variable cv;
        std::deque<std::function<void(factorlib::IFactor&)>> tasks;
        bool stop = false;
        bool joined = false;

        void run() {
            std::unique_lock<std::mutex> lk(mtx);
            while (true) {
                cv.wait(lk, [&] { return stop || !tasks.empty(); });
                if (stop && tasks.empty()) break;
                auto task = std::move(tasks.front());
                tasks.pop_front();
                auto fac = factor;
                lk.unlock();
                if (fac && task) task(*fac);
                lk.lock();
            }
        }
    };

    std::vector<std::unique_ptr<FactorWorker>> g_workers;
#endif

    template <typename Fn>
    void dispatch_to_factors(Fn fn) {
#ifdef FACTORLIB_ENABLE_PARALLEL_INGRESS
        if (g_workers.empty()) return;
        for (auto& worker : g_workers) {
            if (!worker || !worker->factor) continue;
            worker->enqueue(fn);
        }
#else
        for (auto& f : g_factors) {
            if (f) fn(*f);
        }
#endif
    }
}

namespace factorlib::bridge {

    void set_factors(const std::vector<std::shared_ptr<factorlib::IFactor>>& factors) {
#ifdef FACTORLIB_ENABLE_PARALLEL_INGRESS
        for (auto& worker : g_workers) {
            if (worker) worker->shutdown();
        }
        g_workers.clear();
#endif
        g_factors = factors;
#ifdef FACTORLIB_ENABLE_PARALLEL_INGRESS
        g_workers.reserve(g_factors.size());
        for (auto& f : g_factors) {
            g_workers.emplace_back(std::make_unique<FactorWorker>(f));
        }
#endif
    }

    void ingest_snapshot_sh(const std::vector<std_SnapshotStockSH>& v) {
        for (const auto& s : v) {
            auto q = factorlib::DataAdapter::from_snapshot_sh(s);
            // 使用时间戳作为唯一 ID 用于追踪
            uint64_t unique_id = static_cast<uint64_t>(q.data_time_ms);
            FACTORLIB_TRACE_EVENT("ingress", "snapshot_sh", q.instrument_id, 0, unique_id);
            dispatch_to_factors([q](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_snapshot_sz(const std::vector<std_SnapshotStockSZ>& v) {
        for (const auto& s : v) {
            auto q = factorlib::DataAdapter::from_snapshot_sz(s);
            uint64_t unique_id = static_cast<uint64_t>(q.data_time_ms);
            FACTORLIB_TRACE_EVENT("ingress", "snapshot_sz", q.instrument_id, 0, unique_id);
            dispatch_to_factors([q](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_ont(const std::vector<std_OrdAndExeInfo>& v) {
        for (const auto& x : v) {
            auto tick = factorlib::DataAdapter::to_combined(x);
            uint64_t unique_id = static_cast<uint64_t>(tick.data_time_ms);
            FACTORLIB_TRACE_EVENT("ingress", "ont_tick", tick.instrument_id, 0, unique_id);
            dispatch_to_factors([tick](factorlib::IFactor& f){ f.on_tick(tick); });
        }
    }

    void ingest_kline(const std::vector<std_BasicandEnhanceKLine>& v) {
        for (const auto& k : v) {
            auto b = factorlib::DataAdapter::from_kline(k);
            uint64_t unique_id = static_cast<uint64_t>(b.data_time_ms);
            FACTORLIB_TRACE_EVENT("ingress", "kline", b.instrument_id, 0, unique_id);
            dispatch_to_factors([b](factorlib::IFactor& f){ f.on_bar(b); });
        }
    }

    // Overloads for already-converted types (used by tests to avoid dependency on external SDK types)
    void ingest_snapshot_sh(const std::vector<factorlib::QuoteDepth>& v) {
        for (const auto& q : v) {
            uint64_t unique_id = static_cast<uint64_t>(q.data_time_ms);
            FACTORLIB_TRACE_EVENT("ingress", "snapshot_sh", q.instrument_id, 0, unique_id);
            dispatch_to_factors([q](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_snapshot_sz(const std::vector<factorlib::QuoteDepth>& v) {
        for (const auto& q : v) {
            uint64_t unique_id = static_cast<uint64_t>(q.data_time_ms);
            FACTORLIB_TRACE_EVENT("ingress", "snapshot_sz", q.instrument_id, 0, unique_id);
            dispatch_to_factors([q](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_ont(const std::vector<factorlib::CombinedTick>& v) {
        for (const auto& t : v) {
            uint64_t unique_id = static_cast<uint64_t>(t.data_time_ms);
            FACTORLIB_TRACE_EVENT("ingress", "ont_tick", t.instrument_id, 0, unique_id);
            dispatch_to_factors([t](factorlib::IFactor& f){ f.on_tick(t); });
        }
    }

    void ingest_kline(const std::vector<factorlib::Bar>& v) {
        for (const auto& b : v) {
            uint64_t unique_id = static_cast<uint64_t>(b.data_time_ms);
            FACTORLIB_TRACE_EVENT("ingress", "kline", b.instrument_id, 0, unique_id);
            dispatch_to_factors([b](factorlib::IFactor& f){ f.on_bar(b); });
        }
    }

} // namespace factorlib::bridge
