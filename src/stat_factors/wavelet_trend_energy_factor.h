// src/stat_factors/wavelet_trend_energy_factor.h
#pragma once
/**
 * @file wavelet_trend_energy_factor.h
 * @brief 【价-小波趋势能量主导因子】
 *
 * 定义：对价格序列做 MODWT（最大重叠离散小波变换）多尺度分解，
 *      计算“趋势能量占比” R = (∑_{j=j_t..J} E_j) / (∑_{j=1..J} E_j)。
 *      其中 E_j 是尺度 j 的 detail 系数能量（平方和）；j_t 为“趋势起始层”，一般取 4~6。
 *
 * 设计动机：趋势在较粗尺度（j 较大）上占能量比越高，越说明趋势主导。该指标对高频噪声鲁棒。
 * 高频友好：使用 include/math/modwt.h 的 RollingMODWT 滑窗增量实现，O(1) 维护能量环缓冲。
 *
 * 可配置参数（ini: [wave_trend].*）：
 *  - window_size   (int)  滑窗长度 W（Bar 数）
 *  - levels_J      (int)  小波分解层数 J
 *  - trend_start_j (int)  趋势起点层 j_t
 *  - wavelet       (str)  'db4' 或 'sym4'
 *  - debug_mode    (bool) 调试日志
 *
 * 发布主题：wave_trend/energy_ratio  类型 double，值域 [0,1]（窗口预热前不发布）。
 */

#include <string>
#include <unordered_map>
#include <memory>

#include "ifactor.h"
#include "utils/types.h"
#include "utils/databus.h"
#include "utils/log.h"
#include "../config/runtime_config.h"
#include "math/modwt.h"
#include "math/bad_value_policy.h"

namespace factorlib {

struct WaveletTrendConfig {
    int    window_size   = 256;
    int    levels_J      = 6;
    int    trend_start_j = 4;
    std::string wavelet  = "db4";
    bool   debug_mode    = false;
};

static inline constexpr const char* TOP_WT_ENERGY = "wave_trend/energy_ratio";

class WaveletTrendEnergyFactor : public BaseFactor {
public:
    explicit WaveletTrendEnergyFactor(const std::vector<std::string>& codes,
                                      const WaveletTrendConfig& cfg = {})
        : BaseFactor("wavelet_trend_energy", codes), _cfg(cfg) {
        // 允许从 ini 覆盖
        _cfg.window_size   = config::RC().geti("wave_trend.window_size",   _cfg.window_size);
        _cfg.levels_J      = config::RC().geti("wave_trend.levels_J",      _cfg.levels_J);
        _cfg.trend_start_j = config::RC().geti("wave_trend.trend_start_j", _cfg.trend_start_j);
        _cfg.wavelet       = config::RC().get("wave_trend.wavelet",        _cfg.wavelet);
        _cfg.debug_mode    = config::RC().getb("wave_trend.debug_mode",    _cfg.debug_mode);
    }

    static void register_topics(size_t capacity = 120) {
        DataBus::instance().register_topic<double>(TOP_WT_ENERGY, capacity);
    }

    void on_quote(const QuoteDepth& /*q*/) override {}
    void on_tick (const CombinedTick& /*x*/) override {}
    void on_bar  (const Bar& b) override {
        const std::string& code = b.instrument_id;
        ensure_code(code);
        auto* st = _states[code].get();
        const double price = b.close;
        if (!std::isfinite(price)) return;
        st->modwt->push(price);
        if (st->modwt->ready()) {
            const double r = st->modwt->trend_energy_ratio(_cfg.trend_start_j);
            if (std::isfinite(r)) {
                DataBus::instance().publish<double>(TOP_WT_ENERGY, code, b.data_time_ms, r);
                if (_cfg.debug_mode) SPDLOG_DEBUG("[{}] {} @{} R={:.6f}", TOP_WT_ENERGY, code, b.data_time_ms, r);
            }
        }
    }

    bool force_flush(const std::string& /*code*/) override { return false; }

protected:
    void on_code_added(const std::string& code) override {
        (void)code;
        // 在真正首次使用到该 code 时，构造其 modwt（放在 ensure_code 内部）
    }

private:
    struct CodeState {
        std::unique_ptr<math::RollingMODWT<double, math::NoCheckBadValuePolicy>> modwt;
    };
    std::unordered_map<std::string, std::unique_ptr<CodeState>> _states;
    WaveletTrendConfig _cfg;

    void ensure_code(const std::string& code) {
        if (_states.find(code) != _states.end()) return;
        auto st = std::make_unique<CodeState>();
        // 选择小波基
        math::WaveletFilter wf =
            (_cfg.wavelet == "sym4") ? math::wavelet_sym4() : math::wavelet_db4();
        st->modwt = std::make_unique< math::RollingMODWT<double, math::NoCheckBadValuePolicy> >(
            static_cast<std::size_t>(_cfg.window_size),
            _cfg.levels_J,
            wf
        );
        _states.emplace(code, std::move(st));
    }
};

} // namespace factorlib
