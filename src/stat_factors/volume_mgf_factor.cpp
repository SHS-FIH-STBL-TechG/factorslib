#include "volume_mgf_factor.h"

#include <cmath>
#include "../config/runtime_config.h"
#include "utils/config_utils.h"
#include "utils/trace_helper.h"

using factorlib::config::RC;

namespace factorlib {

// =====================[ 构造 & 配置 ]=====================

VolumeMGFFactor::VolumeMGFFactor(const std::vector<std::string>& codes,
                                 const VolumeMGFConfig& cfg)
    : BaseFactor("VolumeMGFFactor", codes)
    , _cfg(cfg) {

    int ws = RC().geti("vol_mgf.window_size", _cfg.window_size);
    if (ws <= 1) {
        LOG_WARN("VolumeMGFFactor: window_size={} 无效，重置为 128", ws);
        ws = 128;
    }
    _cfg.window_size = ws;
    _window_sizes = factorlib::config::load_window_sizes("vol_mgf", _cfg.window_size);
    clamp_window_list(_window_sizes, "[vol_mgf] window_sizes");

    double t = RC().getd("vol_mgf.t", _cfg.t);
    if (!(t > 0.0)) {
        LOG_WARN("VolumeMGFFactor: t={} 无效，重置为 0.01", t);
        t = 0.01;
    }
    _cfg.t = t;
}

// =====================[ 注册 topic ]=====================

void VolumeMGFFactor::register_topics(size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_VOL_MGF, capacity);
}

// =====================[ code 初始化 ]=====================

VolumeMGFFactor::CodeState& VolumeMGFFactor::ensure_state(const ScopeKey& scope) {
    auto key = scope.as_bus_code();
    auto it = _states.find(key);
    if (it == _states.end()) {
        VolumeMGFConfig cfg = _cfg;
        cfg.window_size = scope.window > 0 ? scope.window : _cfg.window_size;
        it = _states.emplace(std::move(key), CodeState(cfg)).first;
    }
    return it->second;
}

void VolumeMGFFactor::on_code_added(const std::string& code) {
    for (int window : _window_sizes) {
        (void)ensure_state(ScopeKey{code, window});
    }
}

// =====================[ 统一事件入口 ]=====================

void VolumeMGFFactor::on_volume_event(const std::string& code_raw,
                                      int64_t ts_ms,
                                      double volume) {
    if (!(volume > 0.0)) {
        return;
    }

    // 为数据入口创建追踪事件（使用时间戳作为唯一 ID）
    uint64_t unique_id = static_cast<uint64_t>(ts_ms);

    ensure_code(code_raw);
    for_each_scope(code_raw, _window_sizes, ts_ms, [&](const ScopeKey& scope) {
        // 为每个窗口的计算创建追踪作用域
        FACTORLIB_TRACE_SCOPE("factor_compute", "VolumeMGF", code_raw, scope.window, unique_id);

        auto& S = ensure_state(scope);
        const std::string scoped_code = scope.as_bus_code();

    // 初始化 max_size（允许在运行时调整 window_size 后，旧 state 重建）
        int window_size = scope.window > 0 ? scope.window : _cfg.window_size;
        if (S.max_size != window_size) {
            S.max_size = window_size;
        S.window.clear();
        S.sum_exp = 0.0;
    }

        double e_new = std::exp(_cfg.t * volume);

        if (S.max_size > 0 && static_cast<int>(S.window.size()) == S.max_size) {
            double v_old = S.window.front();
            S.window.pop_front();
            double e_old = std::exp(_cfg.t * v_old);
            S.sum_exp -= e_old;
        }

    S.window.push_back(volume);
    S.sum_exp += e_new;

    if (!S.window.empty()) {
            compute_and_publish(scoped_code, S, ts_ms);

            // 追踪计数器：记录当前窗口大小
            FACTORLIB_TRACE_COUNTER("factor_compute", "window_size", code_raw, scope.window,
                                    static_cast<double>(S.window.size()));
        }
    });
}

void VolumeMGFFactor::compute_and_publish(const std::string& code,
                                          VolumeMGFFactor::CodeState& S,
                                          int64_t ts_ms) {
    if (S.window.empty()) return;

    double N = static_cast<double>(S.window.size());
    double mgf = S.sum_exp / N;

    if (!std::isfinite(mgf)) {
        LOG_DEBUG("VolumeMGFFactor {} ts={} got non-finite MGF, skip", code, ts_ms);
        return;
    }

    LOG_DEBUG("VolumeMGFFactor {} ts={} N={} MGF(t={})={}",
              code, ts_ms, N, _cfg.t, mgf);

    safe_publish<double>(TOP_VOL_MGF, code, ts_ms, mgf);
}

// =====================[ IFactor 接口 ]=====================

void VolumeMGFFactor::on_quote(const QuoteDepth& q) {
    double v = static_cast<double>(q.volume);
    on_volume_event(q.instrument_id, q.data_time_ms, v);
}

void VolumeMGFFactor::on_tick(const CombinedTick& x) {
    double v = static_cast<double>(x.volume);
    if (v <= 0.0) return;
    on_volume_event(x.instrument_id, x.data_time_ms, v);
}

void VolumeMGFFactor::on_bar(const Bar& b) {
    double v = static_cast<double>(b.volume);
    if (v <= 0.0) return;
    on_volume_event(b.instrument_id, b.data_time_ms, v);
}

} // namespace factorlib
