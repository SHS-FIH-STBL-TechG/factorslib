// src/gaussian_copula_factor.cpp
#include "gaussian_copula_factor.h"
#include <Eigen/Dense>

#include "utils/databus.h"
#include "utils/log.h"
#include "utils/trading_time.h"
#include "../config/runtime_config.h"
#include "math/distributions.h"
#include "math/incremental_rank.h"
#include "math/linear_algebra.h"
#include "math/statistics.h"
#include "utils/config_utils.h"

using factorlib::config::RC;
namespace factorlib {

// 主题常量（与 demo 集成一致）
static const char* TOP_PREDICTION = "gaussian_copula/prediction";

// ---- 构造 / 注册 ----

GaussianCopulaFactor::GaussianCopulaFactor(const GaussianCopulaConfig& cfg,
                                           std::vector<std::string> codes)
    // ★ 改造点：把 name/codes 交给 BaseFactor 管理（行为不变）
    : BaseFactor("GaussianCopulaFactor", std::move(codes))
    , _cfg(cfg) {
    _cfg.window_size    = RC().geti ("gaussian.window_size",    _cfg.window_size);
    _cfg.regularization = RC().getd ("gaussian.regularization", _cfg.regularization);
    _window_sizes       = factorlib::config::load_window_sizes("gaussian", _cfg.window_size);
    clamp_window_list(_window_sizes, "[gaussian] window_sizes");
    auto freq_cfg = factorlib::config::load_time_frequencies("gaussian");
    if (!freq_cfg.empty()) {
        clamp_frequency_list(freq_cfg, "[gaussian] time_frequencies");
        set_time_frequencies_override(freq_cfg);
    }
}

void GaussianCopulaFactor::register_topics(size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_PREDICTION, capacity);
}

// ---- 初始化保障 ----

GaussianCopulaFactor::CodeState& GaussianCopulaFactor::ensure_state(const ScopeKey& scope) {
    auto key = scope.as_bus_code();
    auto it = _states.find(key);
    if (it == _states.end()) {
        it = _states.emplace(key, CodeState{}).first;
    }
    return it->second;
}

GaussianCopulaFactor::IncrementalState& GaussianCopulaFactor::ensure_incremental(const ScopeKey& scope) {
    auto key = scope.as_bus_code();
    auto it = _incremental_states.find(key);
    if (it == _incremental_states.end()) {
        std::size_t win = static_cast<std::size_t>(scope.window > 0 ? scope.window : _cfg.window_size);
        it = _incremental_states.emplace(key, std::make_unique<IncrementalState>(win)).first;
    }
    return *it->second;
}

// ---- 回调实现（保持你的原逻辑）----

void GaussianCopulaFactor::on_quote(const QuoteDepth& q) {
    BaseFactor::ensure_code(q.instrument_id);
    // 针对每个频率/窗口组合分别更新 Copula 状态
    for_each_scope(q.instrument_id, _window_sizes, q.data_time_ms, [&](const ScopeKey& scope) {
        auto& state = ensure_state(scope);
        auto& inc_state = ensure_incremental(scope);
        const std::string scoped_code = scope.as_bus_code();

        // 计算当前中间价
        double mid_price = (q.bid_price + q.ask_price) / 2.0;

        if (state.has_initial_price) {
            double log_return = std::log(mid_price / state.last_mid_price);

            inc_state.update_data(state.current_ofi, state.current_volume, log_return);

            if (inc_state.is_window_full()) {
                double prediction = compute_conditional_expectation_incremental(scoped_code);
                publish_prediction(scoped_code, prediction, q.data_time_ms);
            } else{
                LOG_DEBUG("GaussianCopulaFactor[{}]: 窗口未满：ofi={}, volume={}, ret={}",
                          scoped_code,
                          inc_state.ofi_rank_calc.size(),
                          inc_state.volume_rank_calc.size(),
                          inc_state.return_rank_calc.size());
            }

            state.current_ofi = 0.0;
            state.current_volume = 0.0;
        } else {
            LOG_DEBUG("GaussianCopulaFactor[{}]: 初始化价格: {}", scoped_code, mid_price);
        }

        state.last_mid_price = mid_price;
        state.has_initial_price = true;
    });
}

    void GaussianCopulaFactor::on_tick(const CombinedTick& x) {
    if (x.kind == CombinedKind::Order) {
        Entrust e{};
        e.instrument_id = x.instrument_id;
        e.data_time_ms  = x.data_time_ms;
        e.main_seq      = x.main_seq;
        e.price         = x.price;
        e.side          = x.side;
        e.volume        = x.volume;
        e.order_id      = x.order_id;

        BaseFactor::ensure_code(e.instrument_id);
        for_each_scope(e.instrument_id, _window_sizes, e.data_time_ms, [&](const ScopeKey& scope) {
            auto& state = ensure_state(scope);
            if (e.side == 1) {
                state.current_ofi += static_cast<double>(e.volume);
            } else if (e.side == -1) {
                state.current_ofi -= static_cast<double>(e.volume);
            }
            state.current_volume += static_cast<double>(e.volume);
        });
    } else {
        (void)x;
    }
}


// ★ 保持原 force_flush 语义：窗口未满 → 返回 false；已满 → 发布并返回 true
bool GaussianCopulaFactor::force_flush(const std::string& code) {
    auto inc_it = _incremental_states.find(code);
    if (inc_it == _incremental_states.end()) {
        LOG_WARN("GaussianCopulaFactor: 强制刷新失败，代码 {} 的增量状态不存在", code);
        return false;
    }
    if (!inc_it->second->is_window_full()) {
        LOG_WARN("GaussianCopulaFactor: 强制刷新失败，代码 {} 的窗口未满", code);
        return false;
    }

    double prediction = compute_conditional_expectation_incremental(code);
    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    publish_prediction(code, prediction, timestamp);

    LOG_DEBUG("GaussianCopulaFactor: 强制刷新成功，代码 {} 的预测值: {}", code, prediction);
    return true;
}

// ---- IncrementalState 保持你的原实现 ----

GaussianCopulaFactor::IncrementalState::IncrementalState(size_t window_size)
    : window_size(window_size),
      ofi_rank_calc(),
      volume_rank_calc(),
      return_rank_calc(),
      cov_calc(window_size) {}

void GaussianCopulaFactor::IncrementalState::update_data(double ofi, double volume, double ret) {
    // 更新增量秩计算器
    ofi_rank_calc.push(ofi, window_size);
    volume_rank_calc.push(volume, window_size);
    return_rank_calc.push(ret, window_size);

    // 三个秩都到窗口大小后，推进一次协方差
    if (ofi_rank_calc.size() >= window_size &&
        volume_rank_calc.size() >= window_size &&
        return_rank_calc.size() >= window_size) {

        // 当前数据点的秩 → 正态分数
        double ofi_rank = ofi_rank_calc.median_rank(ofi);
        double volume_rank = volume_rank_calc.median_rank(volume);
        double return_rank = return_rank_calc.median_rank(ret);

        double z_ofi = math::Distributions<double>::normal_quantile(ofi_rank);
        double z_volume = math::Distributions<double>::normal_quantile(volume_rank);
        double z_return = math::Distributions<double>::normal_quantile(return_rank);

        // 更新协方差
        Eigen::Vector3d normal_score;
        normal_score << z_ofi, z_volume, z_return;
        cov_calc.push(normal_score);
    }
}

bool GaussianCopulaFactor::IncrementalState::is_window_full() const {
    // 三个秩与协方差都达到窗口大小时认为窗口已满（与你测试约束一致）
    return ofi_rank_calc.size() >= window_size &&
           volume_rank_calc.size() >= window_size &&
           return_rank_calc.size() >= window_size &&
           cov_calc.size() >= window_size;
}

// ---- 增量条件期望计算（保持原算法）----

double GaussianCopulaFactor::compute_conditional_expectation_incremental(const std::string& code) {
    auto it = _incremental_states.find(code);
    if (it == _incremental_states.end()) {
        return 0.0;
    }
    auto& inc_state = *it->second;

    if (!inc_state.is_window_full()) {
        LOG_WARN("GaussianCopulaFactor[{}]: 计算条件期望失败，窗口未满", code);
        return 0.0;
    }

    // 使用增量统计量
    auto mean = inc_state.cov_calc.mean();
    auto covariance = inc_state.cov_calc.covariance();

    // 正则化协方差矩阵
    covariance += Eigen::Matrix3d::Identity() * _cfg.regularization;

    // 当前 OFI / Volume 的秩 → 正态分数
    auto& state = _states[code];
    double current_ofi_rank = inc_state.ofi_rank_calc.median_rank(state.current_ofi);
    double current_volume_rank = inc_state.volume_rank_calc.median_rank(state.current_volume);

    double z_ofi_current = math::Distributions<double>::normal_quantile(current_ofi_rank);
    double z_volume_current = math::Distributions<double>::normal_quantile(current_volume_rank);

    // 条件期望（对 Z_return | Z_ofi, Z_volume）
    Eigen::Vector2d condition_values;
    condition_values << z_ofi_current, z_volume_current;

    double conditional_mean = math::LinearAlgebra<double>::conditional_expectation(
        mean, covariance, condition_values, 2);

    // N(0,1) → U(0,1)
    double conditional_probability = 0.5 * (1.0 + std::erf(conditional_mean / std::sqrt(2.0)));

    // 经验逆 CDF（ret 的秩样本）
    auto sorted_returns = inc_state.return_rank_calc.get_sorted_data();
    double predicted_return = math::Distributions<double>::empirical_inverse_cdf(sorted_returns, conditional_probability);

    return predicted_return;
}

// 保留原有的“全量计算”入口（内部复用增量实现）
double GaussianCopulaFactor::compute_conditional_expectation(const std::string& code) {
    return compute_conditional_expectation_incremental(code);
}

void GaussianCopulaFactor::publish_prediction(const std::string& code, double prediction, int64_t timestamp) {
    safe_publish<double>(TOP_PREDICTION, code, timestamp, prediction);
}

} // namespace factorlib
