#include "volume_multiscale_autocorr_factor.h"

#include "../config/runtime_config.h"
#include "utils/config_utils.h"

using factorlib::config::RC;

namespace factorlib {

// =====================[ CodeState 实现 ]=====================

VolumeMultiscaleAutocorrFactor::CodeState::CodeState(const VolumeMultiscaleAutocorrConfig& cfg)
    : ready(false)
    , ac1(static_cast<std::size_t>(cfg.window_size), static_cast<std::size_t>(cfg.lag1))
    , ac2(static_cast<std::size_t>(cfg.window_size), static_cast<std::size_t>(cfg.lag2))
    , ac3(static_cast<std::size_t>(cfg.window_size), static_cast<std::size_t>(cfg.lag3))
    , ac4(static_cast<std::size_t>(cfg.window_size), static_cast<std::size_t>(cfg.lag4))
{
    window_size = cfg.window_size;
}

// =====================[ 构造 & 配置 ]=====================

VolumeMultiscaleAutocorrFactor::VolumeMultiscaleAutocorrFactor(
        const std::vector<std::string>& codes,
        const VolumeMultiscaleAutocorrConfig& cfg)
    : BaseFactor("VolumeMultiscaleAutocorr", codes)
    , _cfg(cfg) {

    // 运行时配置覆盖
    int ws = RC().geti("vol_ac.window_size", _cfg.window_size);
    if (ws <= 8) {
        LOG_WARN("VolumeMultiscaleAutocorrFactor: window_size={} 太小，重置为 128", ws);
        ws = 128;
    }
    _cfg.window_size = ws;
    _window_sizes = factorlib::config::load_window_sizes("vol_ac", _cfg.window_size);
    clamp_window_list(_window_sizes, "[vol_ac] window_sizes");
    auto freq_cfg = factorlib::config::load_time_frequencies("vol_ac");
    if (!freq_cfg.empty()) {
        clamp_frequency_list(freq_cfg, "[vol_ac] time_frequencies");
        set_time_frequencies_override(freq_cfg);
    }

    _cfg.lag1 = RC().geti("vol_ac.lag1", _cfg.lag1);
    _cfg.lag2 = RC().geti("vol_ac.lag2", _cfg.lag2);
    _cfg.lag3 = RC().geti("vol_ac.lag3", _cfg.lag3);
    _cfg.lag4 = RC().geti("vol_ac.lag4", _cfg.lag4);

    if (!(_cfg.lag1 > 0 && _cfg.lag2 > 0 && _cfg.lag3 > 0 && _cfg.lag4 > 0)) {
        LOG_WARN("VolumeMultiscaleAutocorrFactor: 部分 lag 配置无效，重置为 1/2/4/8");
        _cfg.lag1 = 1;
        _cfg.lag2 = 2;
        _cfg.lag3 = 4;
        _cfg.lag4 = 8;
    }
}

// =====================[ 注册 topic ]=====================

void VolumeMultiscaleAutocorrFactor::register_topics(size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_VOL_MULTI_AC, capacity);
}

// =====================[ code 初始化 ]=====================

VolumeMultiscaleAutocorrFactor::CodeState& VolumeMultiscaleAutocorrFactor::ensure_state(const ScopeKey& scope) {
    auto key = scope.as_bus_code();
    auto it = _states.find(key);
    if (it == _states.end()) {
        auto cfg = _cfg;
        cfg.window_size = scope.window > 0 ? scope.window : _cfg.window_size;
        it = _states.emplace(std::move(key), CodeState(cfg)).first;
    }
    return it->second;
}

void VolumeMultiscaleAutocorrFactor::on_code_added(const std::string& code) {
    const auto& freqs = get_time_frequencies();
    for (auto freq : freqs) {
        for (int window : _window_sizes) {
            (void)ensure_state(ScopeKey{code, freq, window});
        }
    }
}

// =====================[ 统一事件入口 ]=====================

void VolumeMultiscaleAutocorrFactor::on_volume_event(const std::string& code_raw,
                                                     int64_t ts_ms,
                                                     double volume) {
    if (!(volume > 0.0)) {
        // 空/异常体量直接忽略，不推进窗口
        return;
    }

    ensure_code(code_raw);
    for_each_scope(code_raw, _window_sizes, ts_ms, [&](const ScopeKey& scope) {
        auto& S = ensure_state(scope);
        const std::string scoped_code = scope.as_bus_code();

    S.ready = true;
    (void)S.ac1.push(volume);
    (void)S.ac2.push(volume);
    (void)S.ac3.push(volume);
    (void)S.ac4.push(volume);

    if (S.ac1.ready() && S.ac2.ready() && S.ac3.ready() && S.ac4.ready()) {
            compute_and_publish(scoped_code, S, ts_ms);
        }
    });
}

void VolumeMultiscaleAutocorrFactor::compute_and_publish(const std::string& code,
                                                         VolumeMultiscaleAutocorrFactor::CodeState& S,
                                                         int64_t ts_ms) {
    double r1 = S.ac1.value();
    double r2 = S.ac2.value();
    double r3 = S.ac3.value();
    double r4 = S.ac4.value();

    // 简单平均，多尺度一致性
    double multi = 0.25 * (r1 + r2 + r3 + r4);

    LOG_DEBUG("VolMultiAC {} ts={} r1={} r2={} r3={} r4={} multi={}",
              code, ts_ms, r1, r2, r3, r4, multi);

    safe_publish<double>(TOP_VOL_MULTI_AC, code, ts_ms, multi);
}

// =====================[ IFactor 接口 ]=====================

void VolumeMultiscaleAutocorrFactor::on_quote(const QuoteDepth& q) {
    double v = static_cast<double>(q.volume);
    on_volume_event(q.instrument_id, q.data_time_ms, v);
}

void VolumeMultiscaleAutocorrFactor::on_tick(const CombinedTick& x) {
    double v = static_cast<double>(x.volume);
    if (v <= 0.0) return;
    on_volume_event(x.instrument_id, x.data_time_ms, v);
}

void VolumeMultiscaleAutocorrFactor::on_bar(const Bar& b) {
    double v = static_cast<double>(b.volume);
    if (v <= 0.0) return;
    on_volume_event(b.instrument_id, b.data_time_ms, v);
}

} // namespace factorlib
