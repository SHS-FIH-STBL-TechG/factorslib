// src/bridge/ingress.cpp
#include "bridge/ingress.h"
#include "utils/data_adapter.h"
#include "ifactor.h"
#include "utils/types.h"
#include "utils/processing_axes.h"

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

    void set_time_frequencies(const std::vector<int64_t>& freqs) {
        // demo 层可以在初始化时调用此函数，把“计算频率轴”注入给所有因子
        factorlib::set_time_frequencies(freqs);
    }

    void ingest_snapshot_sh(const std::vector<std_SnapshotStockSH>& v) {
        for (const auto& s : v) {
            auto q = factorlib::DataAdapter::from_snapshot_sh(s);
            dispatch_to_factors([q](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_snapshot_sz(const std::vector<std_SnapshotStockSZ>& v) {
        for (const auto& s : v) {
            auto q = factorlib::DataAdapter::from_snapshot_sz(s);
            dispatch_to_factors([q](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_ont(const std::vector<std_OrdAndExeInfo>& v) {
        for (const auto& x : v) {
            auto tick = factorlib::DataAdapter::to_combined(x);
            dispatch_to_factors([tick](factorlib::IFactor& f){ f.on_tick(tick); });
        }
    }

    void ingest_kline(const std::vector<std_BasicandEnhanceKLine>& v) {
        for (const auto& k : v) {
            auto b = factorlib::DataAdapter::from_kline(k);
            dispatch_to_factors([b](factorlib::IFactor& f){ f.on_bar(b); });
        }
    }

    // Overloads for already-converted types (used by tests to avoid dependency on external SDK types)
    void ingest_snapshot_sh(const std::vector<factorlib::QuoteDepth>& v) {
        for (const auto& q : v) {
            dispatch_to_factors([q](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_snapshot_sz(const std::vector<factorlib::QuoteDepth>& v) {
        for (const auto& q : v) {
            dispatch_to_factors([q](factorlib::IFactor& f){ f.on_quote(q); });
        }
    }

    void ingest_ont(const std::vector<factorlib::CombinedTick>& v) {
        for (const auto& t : v) {
            dispatch_to_factors([t](factorlib::IFactor& f){ f.on_tick(t); });
        }
    }

} // namespace factorlib::bridge
