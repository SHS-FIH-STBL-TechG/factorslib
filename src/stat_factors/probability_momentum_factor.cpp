#include "probability_momentum_factor.h"

#include <cmath>
#include <limits>

#include "utils/databus.h"
#include "utils/log.h"
#include "math/numeric_utils.h"
#include "math/distributions.h"
#include "../config/runtime_config.h"
#include "utils/config_utils.h"

/**
 * @file probability_momentum_factor.cpp
 *
 * 概率动量因子（ProbMomentumFactor）说明：
 * -----------------------------------------
 * 1. 因子背景
 *    - 依据最近一段时间的对数收益序列，假设收益服从带权正态分布；
 *    - 通过加权滑动窗口（权重=成交量）计算收益的均值 μ 和标准差 σ；
 *    - 使用标准正态 CDF 估计 “下一笔收益为正” 的概率 F = Φ(μ/σ)，并根据买卖阈值给出信号；
 *    - 支持 Quote/Tick/Bar 三种数据源，统一走 on_price_event 管道。
 *
 * 2. 计算流程（均摊时间复杂度 O(1)）
 *    - 每到一条价格事件：
 *        a) 计算当前收益 r = ln(price / last_price) 并乘以成交量 w（无量则置 1）；
 *        b) 将 (r, w) 推入 WeightedSlidingWindowStats，维护 sum_w / sum_wr / sum_wr2；
 *        c) 窗口未满直接返回；窗口满后即可在 O(1) 时间求均值、方差与概率；
 *        d) 根据概率与配置的买卖阈值输出信号（+1/-1/0）并发布到 DataBus。
 *
 * 3. 可配置项（来自 runtime_config.ini -> [prob_mom]）：
 *        - window_size      : 滑动窗口长度 N，决定参与统计的样本数；
 *        - window_sizes     : 可选多窗口列表（逗号分隔），不填则使用 window_size；
 *        - time_frequencies : 计算频率（毫秒/秒/分钟等，可多档），默认继承全局频率；
 *        - min_total_weight : 最小权重阈值，若 sum_w < 阈值则不发布结果；
 *        - min_sigma        : σ 的下限，避免除零；
 *        - buy_threshold    : F >= buy_threshold 触发买入信号；
 *        - sell_threshold   : F <= sell_threshold 触发卖出信号；
 *      其余参数（日志开关、频率/窗口上限等）由 BaseFactor 或 CMake 控制。
 *
 * 4. 复杂度与注意事项
 *        - 每个事件的处理仅涉及常数次算术操作，时间复杂度 O(1)，适合高频场景；
 *        - 通过 BaseFactor 的 for_each_scope 支持多频率、多窗口组合；
 *        - 在窗口未满前不会发布概率/信号（而是先累计样本），以保证统计意义；
 *        - `WeightedSlidingWindowStats` 会自动丢弃 NaN 权重、限制窗口容量，并采用 BadValuePolicy 处理异常收益。
 */

using factorlib::config::RC;

namespace factorlib {

using factorlib::math::NumericUtils;
using factorlib::math::Distributions;

// =====================[ 构造 & 配置 ]=====================

/**
 * @brief 构造函数：读取配置并同步多窗口/多频率设定。
 * @param codes  关注的代码列表
 * @param cfg    默认配置（若 runtime_config 有覆盖则以覆盖为准）
 *
 *  主要完成事项：
 *    1) 解析 window_size / window_sizes；
 *    2) 解析 time_frequencies（若指定则覆盖 BaseFactor 的默认频率）；
 *    3) 校验 min_total_weight / min_sigma / buy/sell 阈值；
 *    4) 记录最终配置供后续事件处理使用。
 */
ProbabilityMomentumFactor::ProbabilityMomentumFactor(
        const std::vector<std::string>& codes,
        const ProbMomentumConfig& cfg)
    : BaseFactor("ProbMomentumFactor", codes)
    , _cfg(cfg) {

    // 从运行时配置读取参数（带默认值）
    int ws = RC().geti("prob_mom.window_size", _cfg.window_size);
    if (ws <= 1) {
        LOG_WARN("ProbabilityMomentumFactor: window_size={} 无效，重置为 60", ws);
        ws = 60;
    }
    _cfg.window_size = ws;
    _window_sizes = factorlib::config::load_window_sizes("prob_mom", _cfg.window_size);
    clamp_window_list(_window_sizes, "[prob_mom] window_sizes");
    auto freq_cfg = factorlib::config::load_time_frequencies("prob_mom");
    if (!freq_cfg.empty()) {
        clamp_frequency_list(freq_cfg, "[prob_mom] time_frequencies");
        set_time_frequencies_override(freq_cfg);
    }

    double min_w = RC().getd("prob_mom.min_total_weight", _cfg.min_total_weight);
    if (!(min_w > 0.0)) {
        LOG_WARN("ProbabilityMomentumFactor: min_total_weight={} 无效，重置为 1.0", min_w);
        min_w = 1.0;
    }
    _cfg.min_total_weight = min_w;

    double min_sigma = RC().getd("prob_mom.min_sigma", _cfg.min_sigma);
    if (!(min_sigma > 0.0)) {
        LOG_WARN("ProbabilityMomentumFactor: min_sigma={} 无效，重置为 1e-6", min_sigma);
        min_sigma = 1e-6;
    }
    _cfg.min_sigma = min_sigma;

    double buy_thr  = RC().getd("prob_mom.buy_threshold",  _cfg.buy_threshold);
    double sell_thr = RC().getd("prob_mom.sell_threshold", _cfg.sell_threshold);
    if (!(sell_thr < buy_thr)) {
        LOG_WARN("ProbabilityMomentumFactor: 阈值配置异常 (sell_threshold={} >= buy_threshold={})，使用默认 0.3/0.7",
                 sell_thr, buy_thr);
        sell_thr = 0.3;
        buy_thr  = 0.7;
    }
    _cfg.buy_threshold  = buy_thr;
    _cfg.sell_threshold = sell_thr;
}

// =====================[ topic 注册 ]=====================

/**
 * @brief 注册 DataBus 的输出主题。
 * @details 因子产生两个topic：
 *          1) TOP_PROB_MOM_PROB   : double，表示上涨概率；
 *          2) TOP_PROB_MOM_SIGNAL : double，表示交易信号（-1/0/1）。
 *          统一在测试入口和生产入口调用一次即可。
 */
void ProbabilityMomentumFactor::register_topics(size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_PROB_MOM_PROB,   capacity);
    bus.register_topic<double>(TOP_PROB_MOM_SIGNAL, capacity);
}

// =====================[ 内部辅助 ]=====================

/**
 * @brief 根据 scope（code + freq + window）构造/查找状态。
 * @details 每个 scope 拥有独立的窗口大小与统计器，避免互相干扰。
 *          如果 scope 还未初始化，则在这里分配 SlidingWindowStats。
 */
ProbabilityMomentumFactor::CodeState& ProbabilityMomentumFactor::ensure_state(const ScopeKey& scope) {
    auto key = scope.as_bus_code();
    auto it = _states.find(key);
    if (it == _states.end()) {
        CodeState st;
        // 每个 scope 都可以拥有独立窗口，默认回落到 _cfg.window_size
        st.window_size = scope.window > 0 ? scope.window : _cfg.window_size;
        // SlidingWindowStats 需要知道窗口容量，因此这里重新 reset
        st.stats.reset(static_cast<std::size_t>(st.window_size));
        it = _states.emplace(std::move(key), std::move(st)).first;
        return it->second;
    }
    return it->second;
}

/**
 * @brief BaseFactor 钩子：首次注册 code 时，为所有频率/窗口组合预建状态。
 * @details 这样可以避免运行时首次遍历时的 map 扩容，也方便在测试里直接读取。
 */
void ProbabilityMomentumFactor::on_code_added(const std::string& code) {
    // 预先为每个 (code, freq, window) 初始化 state，可避免运行时首次访问的锁竞争
    const auto& freqs = get_time_frequencies();
    for (auto freq : freqs) {
        for (int window : _window_sizes) {
            (void)ensure_state(ScopeKey{code, freq, window});
        }
    }
}

/**
 * @brief 在窗口满足条件后，计算上涨概率并发布信号。
 * @details 计算要点：
 *          1) 使用 WeightedSlidingWindowStats 的均值/方差；
 *          2) 若总权重不足或 sigma 不可用，则直接返回；
 *          3) 概率 F = Φ(μ/σ)，信号依据 buy/sell 阈值映射为 1/0/-1。
 * @param code    scope 的完整 code（包含 freq/window）
 * @param S       对应 scope 的状态
 * @param ts_ms   最近一条事件时间戳
 */
void ProbabilityMomentumFactor::compute_and_publish(
        const std::string& code,
        CodeState& S,
        int64_t ts_ms) {

    double mu = 0.0;
    double sigma = 0.0;

    if (!S.stats.mean_var(mu, sigma)) {
        return; // 样本不足或权重为 0
    }

    if (!(S.stats.sum_w() >= _cfg.min_total_weight)) {
        return; // 有效权重不足
    }

    if (!(sigma > 0.0)) {
        sigma = _cfg.min_sigma;
    }
    if (!(sigma > 0.0)) {
        LOG_DEBUG("ProbabilityMomentumFactor: sigma 非正, code={}", code);
        return;
    }

    // 正态分布下 P(R > 0) = Φ(mu / sigma)
    double z = mu / sigma;
    if (!std::isfinite(z)) {
        LOG_DEBUG("ProbabilityMomentumFactor: 非有限 z, code={}, z={}", code, z);
        return;
    }

    double F = Distributions<>::normal_cdf(z);
    if (!std::isfinite(F)) {
        LOG_DEBUG("ProbabilityMomentumFactor: 非有限 F, code={}, F={}", code, F);
        return;
    }

    // 生成信号：-1, 0, +1
    double signal = 0.0;
    if (F >= _cfg.buy_threshold) {
        signal = 1.0;
    } else if (F <= _cfg.sell_threshold) {
        signal = -1.0;
    }

    // 发布
    safe_publish<double>(TOP_PROB_MOM_PROB,   code, ts_ms, F);
    safe_publish<double>(TOP_PROB_MOM_SIGNAL, code, ts_ms, signal);
}

// 统一价格事件入口
/**
 * @brief 将 Quote/Tick/Bar 的价格事件统一处理。
 * @details 对每个 (code,freq,window) scope：
 *          1) 计算收益 r，与成交量 volume 作为权重；
 *          2) 将 (r,w) 推入滑动窗口；
 *          3) 若窗口未满，直接返回；满窗则计算概率并发布。
 */
void ProbabilityMomentumFactor::on_price_event(
        const std::string& code_raw,
        int64_t ts_ms,
        double price,
        double volume) {

    if (!(price > 0.0)) return;

    ensure_code(code_raw);
    // 遍历“相同 code 下所有频率 + 窗口”的组合，每个组合都会独立维护一份统计
    for_each_scope(code_raw, _window_sizes, ts_ms, [&](const ScopeKey& scope) {
        auto& S = ensure_state(scope);
        const std::string scoped_code = scope.as_bus_code();

        if (!S.has_last_price) {
            S.has_last_price = true;
            S.last_price     = price;
            return;
        }

        if (!(S.last_price > 0.0) || !std::isfinite(S.last_price)) {
            S.last_price = price;
            return;
        }

        // 对数收益率，使用通用工具
        double r = NumericUtils<double>::log_return(price, S.last_price);
        if (!std::isfinite(r)) {
            S.last_price = price;
            return;
        }

        S.last_price = price;

        // 成交量权重。为防止 0 影响，设个最小权重 1.0
        double w = (volume > 0.0 ? volume : 1.0);

        S.stats.push(r, w);
        if (S.stats.size() < static_cast<std::size_t>(S.window_size)) return;
        compute_and_publish(scoped_code, S, ts_ms);
    });
}

// =====================[ 事件入口 ]=====================

/** @brief Quote 事件：使用中间价作为价格，volume 直接引用 quote.volume。 */
void ProbabilityMomentumFactor::on_quote(const QuoteDepth& q) {
    double bid = q.bid_price;
    double ask = q.ask_price;
    if (!(bid > 0.0) || !(ask > 0.0)) return;

    double mid = 0.5 * (bid + ask);
    if (!std::isfinite(mid) || mid <= 0.0) return;

    double vol = static_cast<double>(q.volume);
    on_price_event(q.instrument_id, q.data_time_ms, mid, vol);
}

/** @brief Tick 事件：直接使用成交价，volume 取 CombinedTick.volume。 */
void ProbabilityMomentumFactor::on_tick(const CombinedTick& x) {
    if (!(x.price > 0.0)) return;

    double vol = static_cast<double>(x.volume);
    on_price_event(x.instrument_id, x.data_time_ms, x.price, vol);
}

/** @brief Bar 事件：使用收盘价和 bar.volume 作为输入。 */
void ProbabilityMomentumFactor::on_bar(const Bar& b) {
    if (!(b.close > 0.0)) return;

    double vol = static_cast<double>(b.volume);
    on_price_event(b.instrument_id, b.data_time_ms, b.close, vol);
}

} // namespace factorlib
