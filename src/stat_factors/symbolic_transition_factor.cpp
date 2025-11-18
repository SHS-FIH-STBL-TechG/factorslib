#include "symbolic_transition_factor.h"
#include <cmath>

using factorlib::config::RC;

namespace factorlib {

SymbolicTransitionFactor::SymbolicTransitionFactor(const std::vector<std::string>& codes,
                                                   const SymbolicCfg& cfg)
    : BaseFactor("SymbolicTransition", codes), _cfg(cfg) {
    _cfg.window_size = RC().geti("symbolic.window_size", _cfg.window_size);
    _cfg.symbols_k   = RC().geti("symbolic.symbols_k",   _cfg.symbols_k);
}


void SymbolicTransitionFactor::ensure_state(const std::string& code) {
    if (_states.find(code) != _states.end()) return;

    CodeState st;
    // 创建符号化边界（k等分分箱，这里使用简单的-0.1到0.1范围）
    std::vector<double> edges;
    double min_val = -0.1, max_val = 0.1;
    for (int i = 0; i <= _cfg.symbols_k; ++i) {
        edges.push_back(min_val + i * (max_val - min_val) / _cfg.symbols_k);
    }

    math::Symbolizer<double> symbolizer(edges);
    st.sym = std::make_unique<math::RollingSymbolicDynamics<double>>(
        static_cast<std::size_t>(_cfg.window_size),
        symbolizer  // 传递 Symbolizer 对象而不是整数
    );
    _states.emplace(code, std::move(st));
}

void SymbolicTransitionFactor::on_price_event(const std::string& code, int64_t ts, double price) {
    ensure_state(code);
    auto& s = _states[code];
    if (!std::isfinite(price)) return;

    if (s.has_last && s.last_p>0.0) {
        double r = std::log(price / s.last_p);
        s.sym->push(r);
        maybe_publish(code, ts);
    }
    s.last_p = price; s.has_last = true;
}

void SymbolicTransitionFactor::maybe_publish(const std::string& code, int64_t ts) {
    auto& s = _states[code];
    if (!s.sym->ready()) return;
    // 谱半径近似（RollingSymbolicDynamics 内部已维护转移计数/概率）
    double eig1 = s.sym->leading_eigenvalue();
    if (std::isfinite(eig1)) {
        safe_publish<double>(TOP_SYMBOLIC_EIG1, code, ts, eig1);
        LOG_DEBUG("SymbolicEig1[{}]: k={}, eig1≈{:.6f}", code, _cfg.symbols_k, eig1);
    }
}

void SymbolicTransitionFactor::on_quote(const QuoteDepth& q) {
    double mid = (q.bid_price + q.ask_price) / 2.0;
    on_price_event(q.instrument_id, q.data_time_ms, mid);
}
void SymbolicTransitionFactor::on_tick(const CombinedTick& x) {
    if (x.kind==CombinedKind::Trade) on_price_event(x.instrument_id, x.data_time_ms, x.price);
}
void SymbolicTransitionFactor::on_bar(const Bar& b) {
    on_price_event(b.instrument_id, b.data_time_ms, b.close);
}

} // namespace factorlib
