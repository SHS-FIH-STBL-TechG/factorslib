// src/stat_factors/memory_kernel_decay_factor.h
#pragma once
/**
 * @file memory_kernel_decay_factor.h
 * @brief 【价-价格记忆核衰减因子】
 *
 * 定义：M = ∑_{k=1}^L w_k(α) · ρ(k)，其中 ρ(k) 是滞后 k 的自相关。
 * 直觉：若 M 较大且缓慢衰减，说明序列具“长记忆/惯性”。
 *
 * 实现：使用 include/math/memory_kernel.h 的 MemoryKernelEstimator（滑窗增量）。
 * 输入：使用 Bar.close 的对数收益 r_t = log(close_t / close_{t-1})。
 *
 * 可配置参数（ini: [mem_kernel].*）：
 *  - window_size (int)  自相关估计窗口 W
 *  - L           (int)  记忆阶数（最大滞后）
 *  - alpha       (dbl)  分数阶参数 α ∈ (0,1)
 *  - debug_mode  (bool) 调试日志
 *
 * 发布主题：memory_kernel/decay  类型 double。
 */

#include <string>
#include <unordered_map>
#include <memory>
#include <cmath>

#include "ifactor.h"
#include "utils/types.h"
#include "utils/databus.h"
#include "utils/log.h"
#include "../config/runtime_config.h"
#include "math/memory_kernel.h"
#include "math/rolling_autocorr.h"

namespace factorlib {

struct MemoryKernelConfig {
    int    window_size = 256;
    int    L           = 50;
    double alpha       = 0.6;
    bool   debug_mode  = false;
};

static inline constexpr const char* TOP_MEMK = "memory_kernel/decay";

class MemoryKernelDecayFactor : public BaseFactor {
public:
    explicit MemoryKernelDecayFactor(const std::vector<std::string>& codes,
                                     const MemoryKernelConfig& cfg = {})
        : BaseFactor("memory_kernel_decay", codes), _cfg(cfg) {
        _cfg.window_size = config::RC().geti("mem_kernel.window_size", _cfg.window_size);
        _cfg.L           = config::RC().geti("mem_kernel.L",           _cfg.L);
        _cfg.alpha       = config::RC().getd("mem_kernel.alpha",       _cfg.alpha);
        _cfg.debug_mode  = config::RC().getb("mem_kernel.debug_mode",  _cfg.debug_mode);
    }

    static void register_topics(size_t capacity=120) {
        DataBus::instance().register_topic<double>(TOP_MEMK, capacity);
    }

    void on_quote(const QuoteDepth& /*q*/) override {}
    void on_tick (const CombinedTick& /*x*/) override {}
    void on_bar  (const Bar& b) override {
        const std::string& code = b.instrument_id;
        ensure_code_(code);
        auto* st = _states[code].get();
        double p = b.close;
        if (!std::isfinite(p) || p <= 0) return;
        if (st->last_close > 0) {
            double r = std::log(p / st->last_close);
            st->mk->push(r);
            if (st->mk->ready()) {
                double M = st->mk->value();
                if (std::isfinite(M)) {
                    DataBus::instance().publish<double>(TOP_MEMK, code, b.data_time_ms, M);
                    if (_cfg.debug_mode) SPDLOG_DEBUG("[{}] {} @{} M={:.6f}", TOP_MEMK, code, b.data_time_ms, M);
                }
            }
        }
        st->last_close = p;
    }

    bool force_flush(const std::string& /*code*/) override { return false; }

private:
    struct CodeState {
        std::unique_ptr< math::MemoryKernelEstimator<double> > mk;
        double last_close = 0.0;
    };
    std::unordered_map<std::string, std::unique_ptr<CodeState>> _states;
    MemoryKernelConfig _cfg;

    void ensure_code_(const std::string& code) {
        if (_states.find(code) != _states.end()) return;
        auto st = std::make_unique<CodeState>();
        st->mk = std::make_unique< math::MemoryKernelEstimator<double> >(
            static_cast<std::size_t>(_cfg.window_size),
            static_cast<std::size_t>(_cfg.L),
            _cfg.alpha
        );
        _states.emplace(code, std::move(st));
    }
};

} // namespace factorlib
