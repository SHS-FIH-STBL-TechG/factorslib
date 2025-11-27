#include "factors/stat/granger_causality_factor.h"

#include <algorithm>
#include <cassert>
#include <Eigen/Dense>

#include "config/runtime_config.h"
#include "config/config_utils.h"
#include "utils/trading_time.h"

/**
 * @file granger_causality_factor.cpp
 *
 * 格兰杰因果强度因子说明：
 * -----------------------------------------
 * 1. 因子背景
 *    - 目标是检验“订单流不平衡 OFI 是否格兰杰导致中间价变化”；
 *    - 维护前视/后视两个线性回归模型：
 *          Restricted  : y_t = β0 + ∑_{i=1}^p β_i y_{t-i}
 *          Unrestricted: y_t = β0 + ∑_{i=1}^p β_i y_{t-i} + ∑_{j=1}^q γ_j x_{t-j}
 *    - 通过比较 RSSr/RSSu 构造 F 统计量，再得到 p 值；
 *    - p 值越小，说明 OFI 对价格变动的解释力越强。可选发布 -log10(p) 作为“强度”。
 *
 * 2. 事件流程
 *    - on_tick(Entrust)：累加 pending_ofi（买加卖减）；
 *    - on_quote：计算当前 mid price，并与上一笔 mid 形成 y_t；
 *    - emit_sample_event_driven_：当滑窗足够长时，收集 {y_t, y_lags, x_lags} 推入 OLS；
 *    - push_sample_and_update_：求解 Restricted / Unrestricted，计算 F/p 并发布；
 *    - force_flush 在逐条模式下无意义，固定返回 false。
 *
 * 3. 配置来源（runtime_config.ini -> [granger]）
 *    - window_size / window_sizes     ：滑窗大小，可多窗并行；
 *    - p_lags / q_lags               ：自变量/因变量滞后阶数；
 *    - min_effective                 ：最小有效样本数；
 *    - use_neglog10 / strength_clip  ：-log10(p) 及裁剪控制；
 *    - publish_raw_p                 ：是否同时发布原始 p 值。
 *
 * 4. 实现要点
 *    - 每个 ScopeKey(code|window) 拥有独立的 CodeState，保证多窗口隔离；
 *    - 通过 SlidingNormalEq 增量维护 OLS，既能滑窗退样，又保持数值稳定；
 *    - p 值/强度发布到固定主题，便于下游读取；
 *    - 日志记录 N/F/RSS 等信息，便于调试。
 */
using factorlib::config::RC;
namespace factorlib {

    using Vec = Eigen::VectorXd;

    // CodeState 用于缓存每个 scope（code|window）的回归器/滑窗信息，
    // 构造时即根据配置初始化受限模型（ne_r）与完整模型（ne_u）的维度。
    GrangerCausalityFactor::CodeState::CodeState(const GrangerConfig& cfg, int window)
        : d_r(1 + std::max(0, cfg.p_lags))
        , d_u(1 + std::max(0, cfg.p_lags) + std::max(0, cfg.q_lags))
        , ne_r(d_r, window)
        , ne_u(d_u, window)
        , window_size(window) {}

    // ===== 构造 / 注册 =====
    GrangerCausalityFactor::GrangerCausalityFactor(const std::vector<std::string>& codes,
                                                   const GrangerConfig& cfg)
        : BaseFactor("GrangerCausalityStrength", codes)
        , _cfg(cfg) {
        // —— 运行时覆盖：与 gaussian_copula_factor.cpp 同样的模式，支持运行时热切自定义参数 ——
        _cfg.window_size   = RC().geti ("granger.window_size",   _cfg.window_size);
        _cfg.p_lags        = RC().geti ("granger.p_lags",        _cfg.p_lags);
        _cfg.q_lags        = RC().geti ("granger.q_lags",        _cfg.q_lags);
        _cfg.min_effective = RC().geti ("granger.min_effective", _cfg.min_effective);

        // —— 发布相关：是否输出 -log10(p)、强度裁剪上限等 —— 
        _cfg.use_neglog10  = RC().getb ("granger.use_neglog10",  _cfg.use_neglog10);
        _cfg.strength_clip = RC().getd ("granger.strength_clip", _cfg.strength_clip);
        _window_sizes      = factorlib::config::load_window_sizes("granger", _cfg.window_size);
        clamp_window_list(_window_sizes, "[granger] window_sizes");
    }


    void GrangerCausalityFactor::register_topics(size_t capacity) {
        auto& bus = DataBus::instance();
        // 在任何实例创建之前完成主题注册：
        // TOP_GRANGER_STRENGTH：根据配置发布 -log10(p) 或原始 p；
        // TOP_GRANGER_PVAL    ：始终保留原始 p 值，方便调试/监控。
        bus.register_topic<double>(TOP_GRANGER_STRENGTH, capacity);
        bus.register_topic<double>(TOP_GRANGER_PVAL,     capacity); // 仅注册，不强制发布
    }

    // ===== BaseFactor 钩子：首次见到某个 code =====
    void GrangerCausalityFactor::on_code_added(const std::string& code) {
        for (int window : _window_sizes) {
            (void)ensure_state(ScopeKey{code, window});
        }
    }

    // ===== IFactor 事件入口 =====
    void GrangerCausalityFactor::on_quote(const QuoteDepth& q) {
        BaseFactor::ensure_code(q.instrument_id);
        for (int window : _window_sizes) {
            ScopeKey scope{q.instrument_id, window};
            on_any_event_(scope, q.data_time_ms, q, std::nullopt, std::nullopt);
        }
    }

    void GrangerCausalityFactor::on_tick(const CombinedTick& x) {
        if (x.kind == CombinedKind::Trade) {
            Transaction t{};
            t.instrument_id = x.instrument_id;
            t.data_time_ms  = x.data_time_ms;
            t.main_seq      = x.main_seq;
            t.price         = x.price;
            t.side          = x.side;
            t.volume        = x.volume;
            t.bid_no        = x.bid_no;
            t.ask_no        = x.ask_no;
            BaseFactor::ensure_code(t.instrument_id);
            for (int window : _window_sizes) {
                ScopeKey scope{t.instrument_id, window};
                on_any_event_(scope, t.data_time_ms, std::nullopt, t, std::nullopt);
            }
        } else {
            Entrust e{};
            e.instrument_id = x.instrument_id;
            e.data_time_ms  = x.data_time_ms;
            e.main_seq      = x.main_seq;
            e.price         = x.price;
            e.side          = x.side;
            e.volume        = x.volume;
            e.order_id      = x.order_id;
            BaseFactor::ensure_code(e.instrument_id);
            for (int window : _window_sizes) {
                ScopeKey scope{e.instrument_id, window};
                on_any_event_(scope, e.data_time_ms, std::nullopt, std::nullopt, e);
            }
        }
    }


    // ===== 强制冲桶（事件驱动模式下恒为 false）=====
    bool GrangerCausalityFactor::force_flush(const std::string& code) {
        return false;
    }

    // ===== 内部：确保 code 状态 =====
    GrangerCausalityFactor::CodeState& GrangerCausalityFactor::ensure_state(const ScopeKey& scope) {
        auto key = scope.as_bus_code();
        auto it = _states.find(key);
        if (it == _states.end()) {
            int window = scope.window > 0 ? scope.window : _cfg.window_size;
            it = _states.emplace(key, CodeState(_cfg, window)).first;
        }
        return it->second;
    }

    // ===== 内部：统一事件入口（逐条模式） =====
    void GrangerCausalityFactor::on_any_event_(const ScopeKey& scope, int64_t ts_ms,
                                               const std::optional<QuoteDepth>& qopt,
                                               const std::optional<Transaction>& topt,
                                               const std::optional<Entrust>& eopt) {
        auto& S = ensure_state(scope);
        const std::string scoped_code = scope.as_bus_code();

        // ---------- 逐条模式 ----------
        if (eopt) {
            const auto& e = *eopt;
            S.pending_ofi += (e.side > 0 ? (double)e.volume : -(double)e.volume);
        }

        if (topt) { /* 可选：基于成交方向累计 OFI */ }

        if (qopt) {
            const auto& q = *qopt;
            if (q.bid_price > 0 && q.ask_price > 0) {
                double mid_now = 0.5 * (q.bid_price + q.ask_price);
                if (S.last_mid.has_value()) {
                    double y_t = mid_now - *S.last_mid;
                    emit_sample_event_driven_(scope, ts_ms, y_t);
                }
                S.last_mid = mid_now;
            }
        }
    }

    // ===== 内部：逐条模式在 mid 刷新时产出样本 =====
    void GrangerCausalityFactor::emit_sample_event_driven_(const ScopeKey& scope,
                                                           int64_t ts_ms, double y_t) {
        auto& S = ensure_state(scope);

        double x_t = S.pending_ofi;
        S.pending_ofi = 0.0;

        S.x_win.push_back(x_t);
        S.y_win.push_back(y_t);
        while ((int)S.x_win.size() > S.window_size) S.x_win.pop_front();
        while ((int)S.y_win.size() > S.window_size) S.y_win.pop_front();

        if ((int)S.y_win.size() <= std::max(_cfg.p_lags, _cfg.q_lags)) return;
        std::vector<double> y_lags(_cfg.p_lags), x_lags(_cfg.q_lags);
        for (int i=0;i<_cfg.p_lags;++i) y_lags[i] = S.y_win[(int)S.y_win.size()-1 - (i+1)];
        for (int j=0;j<_cfg.q_lags;++j) x_lags[j] = S.x_win[(int)S.x_win.size()-1 - (j+1)];

        push_sample_and_update_(scope, ts_ms, y_t, y_lags, x_lags);
    }

    // ===== 内部：核心回归 / 统计量 / 发布 =====
    void GrangerCausalityFactor::push_sample_and_update_(const ScopeKey& scope, int64_t ts_ms,
                                                         double y_t,
                                                         const std::vector<double>& y_lags,
                                                         const std::vector<double>& x_lags) {
        auto& S = ensure_state(scope);
        const std::string scoped_code = scope.as_bus_code();
        const int p = _cfg.p_lags;
        const int q = _cfg.q_lags;

        Vec xr(S.d_r);
        xr(0) = 1.0;
        for (int i=0;i<p;++i) xr(1+i) = y_lags[i];

        Vec xu(S.d_u);
        xu(0) = 1.0;
        for (int i=0;i<p;++i) xu(1+i) = y_lags[i];
        for (int j=0;j<q;++j) xu(1+p+j) = x_lags[j];

        S.ne_r.push(xr, y_t);
        S.ne_u.push(xu, y_t);

        const int N   = std::min((int)S.y_win.size(), S.window_size);
        const int df2 = N - (p + q + 1);
        if (N < std::max(_cfg.min_effective, p + q + 4) || df2 <= 0) return;

        Vec br(S.d_r), bu(S.d_u);
        double RSSr=0, RSSu=0;
        if (!S.ne_r.solve(br, RSSr)) return;
        if (!S.ne_u.solve(bu, RSSu)) return;

        const int k = std::max(1, q);
        const double eps = 1e-18; // 分母/差值保护下界

        double diff = RSSr - RSSu;
        if (diff < 0.0) diff = 0.0;                 // 理论上 RSSr>=RSSu，数值误差截断
        double num = diff / static_cast<double>(k);

        int safe_df2 = std::max(1, df2);
        double den = RSSu / static_cast<double>(safe_df2);
        if (!(den > 0.0)) den = eps;                // 分母下界，永不为 0

        const double F = num / den;

        double pval = math::fisher_f_sf<double>(F, k, safe_df2);
        if (!std::isfinite(pval)) {
            // 极端情况下兜底
            pval = (F <= 0.0 ? 1.0 : std::numeric_limits<double>::min());
        }

        publish_strength_(scope, ts_ms, pval);

        LOG_DEBUG("[{}] ts={} N={} F={} p={} RSSr={} RSSu={}",
            scoped_code, ts_ms, N, F, pval, RSSr, RSSu);
    }

    void GrangerCausalityFactor::publish_strength_(const ScopeKey& scope,
                                                   int64_t ts_ms, double p) const {
        const std::string scoped_code = scope.as_bus_code();
        double value = p;
        if (_cfg.use_neglog10) {
            if (p <= 0) p = std::numeric_limits<double>::min();
            value = -std::log10(p);
            if (value > _cfg.strength_clip) value = _cfg.strength_clip;
            if (!std::isfinite(value)) value = 0.0;
        }
        safe_publish<double>(TOP_GRANGER_STRENGTH, scoped_code, ts_ms, value);
        safe_publish<double>(TOP_GRANGER_PVAL,     scoped_code, ts_ms, p);
    }

} // namespace factorlib
