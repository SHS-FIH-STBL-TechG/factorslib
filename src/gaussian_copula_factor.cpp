// src/gaussian_copula_factor.cpp
#include "gaussian_copula_factor.h"
#include <Eigen/Dense>

// 包含新的增量计算工具
#include "utils/math/incremental_rank.h"
#include "utils/math/statistics.h"
#include "utils/math/distributions.h"
#include "utils/math/linear_algebra.h"
#include "utils/trading_time.h"
#include "utils/log.h"  // 添加日志

namespace factorlib {

// 主题常量
static const char* TOP_PREDICTION = "gaussian_copula/prediction";

// ---- GaussianCopulaFactor ----

GaussianCopulaFactor::GaussianCopulaFactor(const GaussianCopulaConfig& cfg, std::vector<std::string> codes)
    : _cfg(cfg), _codes(std::move(codes)) {}

void GaussianCopulaFactor::register_topics(size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_PREDICTION, capacity);
}

void GaussianCopulaFactor::ensure_code(const std::string& code) {
    if (_states.find(code) == _states.end()) {
        _states[code] = CodeState{};
    }

    // 初始化增量计算状态
    if (_incremental_states.find(code) == _incremental_states.end()) {
        _incremental_states[code] = std::make_unique<IncrementalState>(_cfg.window_size);
    }
}

void GaussianCopulaFactor::on_quote(const QuoteDepth& q) {
    ensure_code(q.instrument_id);
    auto& state = _states[q.instrument_id];
    auto& inc_state = _incremental_states[q.instrument_id];

    // 计算当前中间价
    double mid_price = (q.bid_price + q.ask_price) / 2.0;

    if (state.has_initial_price) {
        // 计算对数收益率: log(当前价/上一期价)
        double log_return = std::log(mid_price / state.last_mid_price);

        // if (_cfg.debug_mode) {
        //     LOG_DEBUG("GaussianCopulaFactor[{}]: 收益率计算: 当前价={}, 上期价={}, 收益率={}",
        //              q.instrument_id, mid_price, state.last_mid_price, log_return);
        // }

        // 使用增量计算更新统计量
        inc_state->update_data(state.current_ofi, state.current_volume, log_return);

        // 如果窗口已满，计算因子值
        if (inc_state->is_window_full()) {
            double prediction = compute_conditional_expectation_incremental(q.instrument_id);
            publish_prediction(q.instrument_id, prediction, q.data_time_ms);

            // if (_cfg.debug_mode) {
            //     LOG_DEBUG("GaussianCopulaFactor[{}]: 窗口已满，计算预测值: {}",
            //              q.instrument_id, prediction);
            // }
        } else {
            if (_cfg.debug_mode) {
                LOG_DEBUG("GaussianCopulaFactor[{}]: 窗口未满，当前大小: ofi={}, volume={}, return={}",
                         q.instrument_id,
                         inc_state->ofi_rank_calc.size(),
                         inc_state->volume_rank_calc.size(),
                         inc_state->return_rank_calc.size());
            }
        }

        // 重置当前tick的累计值
        state.current_ofi = 0.0;
        state.current_volume = 0.0;
    } else {
        // 第一次收到行情，设置初始价格
        if (_cfg.debug_mode) {
            LOG_DEBUG("GaussianCopulaFactor[{}]: 初始化价格: {}", q.instrument_id, mid_price);
        }
    }

    // 更新状态
    state.last_mid_price = mid_price;
    state.has_initial_price = true;
}

void GaussianCopulaFactor::on_entrust(const Entrust& e) {
    ensure_code(e.instrument_id);
    auto& state = _states[e.instrument_id];

    // 累计OFI和成交量
    // OFI = 买方订单总量 - 卖方订单总量
    if (e.side == 1) {  // 买方
        state.current_ofi += static_cast<double>(e.volume);
    } else if (e.side == -1) {  // 卖方
        state.current_ofi -= static_cast<double>(e.volume);
    }

    state.current_volume += static_cast<double>(e.volume);
    //
    // if (_cfg.debug_mode) {
    //     LOG_DEBUG("GaussianCopulaFactor[{}]: 委托处理: side={}, volume={}, 当前OFI={}, 当前Volume={}",
    //              e.instrument_id, e.side, e.volume, state.current_ofi, state.current_volume);
    // }
}

void GaussianCopulaFactor::on_transaction(const Transaction& t) {
    // 成交数据可用于验证，但主要计算依赖委托数据
}

bool GaussianCopulaFactor::force_flush(const std::string& code) {
    auto inc_it = _incremental_states.find(code);
    if (inc_it == _incremental_states.end()) {
        LOG_WARN("GaussianCopulaFactor: 强制刷新失败，代码 {} 的状态不存在", code);
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

// ---- IncrementalState 实现 ----

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

    // 如果所有秩计算器都达到窗口大小，更新协方差计算
    if (ofi_rank_calc.size() >= window_size &&
        volume_rank_calc.size() >= window_size &&
        return_rank_calc.size() >= window_size) {

        // 计算当前数据点的正态分数
        double ofi_rank = ofi_rank_calc.median_rank(ofi);
        double volume_rank = volume_rank_calc.median_rank(volume);
        double return_rank = return_rank_calc.median_rank(ret);

        double z_ofi = math::Distributions::normal_quantile(ofi_rank);
        double z_volume = math::Distributions::normal_quantile(volume_rank);
        double z_return = math::Distributions::normal_quantile(return_rank);

        // 更新协方差计算
        Eigen::Vector3d normal_score;
        normal_score << z_ofi, z_volume, z_return;
        cov_calc.push(normal_score);
        }
}

bool GaussianCopulaFactor::IncrementalState::is_window_full() const {
    // 只有当所有秩计算器和协方差计算器都达到窗口大小时才认为窗口已满
    return ofi_rank_calc.size() >= window_size &&
           volume_rank_calc.size() >= window_size &&
           return_rank_calc.size() >= window_size &&
           cov_calc.size() >= window_size;  // 协方差计算器也需要达到窗口大小
}

// ---- 增量条件期望计算 ----

double GaussianCopulaFactor::compute_conditional_expectation_incremental(const std::string& code) {
    auto& inc_state = _incremental_states[code];

    if (!inc_state->is_window_full()) {
        LOG_WARN("GaussianCopulaFactor[{}]: 计算条件期望失败，窗口未满", code);
        return 0.0;
    }

    // 使用增量计算的统计量
    auto mean = inc_state->cov_calc.mean();
    auto covariance = inc_state->cov_calc.covariance();

    // 正则化协方差矩阵
    covariance += Eigen::Matrix3d::Identity() * _cfg.regularization;

    // 计算当前观测值的Z分数（使用增量秩计算器）
    auto& state = _states[code];
    double current_ofi_rank = inc_state->ofi_rank_calc.median_rank(state.current_ofi);
    double current_volume_rank = inc_state->volume_rank_calc.median_rank(state.current_volume);

    double z_ofi_current = math::Distributions::normal_quantile(current_ofi_rank);
    double z_volume_current = math::Distributions::normal_quantile(current_volume_rank);

    // 使用增量计算的协方差矩阵计算条件期望
    Eigen::Vector2d condition_values;
    condition_values << z_ofi_current, z_volume_current;

    double conditional_mean = math::LinearAlgebra<double>::conditional_expectation(
        mean, covariance, condition_values, 2);

    // 转换回均匀分布并应用经验逆CDF
    double conditional_probability = 0.5 * (1.0 + std::erf(conditional_mean / std::sqrt(2.0)));

    // 使用增量秩计算器中的排序数据计算经验逆CDF
    auto sorted_returns = inc_state->return_rank_calc.get_sorted_data();
    double predicted_return = math::Distributions::empirical_inverse_cdf(sorted_returns, conditional_probability);

    // if (_cfg.debug_mode) {
    //     LOG_DEBUG("GaussianCopulaFactor[{}]: OFI={}, Volume={}, PredictedReturn={}",
    //              code, state.current_ofi, state.current_volume, predicted_return);
    // }

    return predicted_return;
}

// 保留原有的全量计算方法作为备选
double GaussianCopulaFactor::compute_conditional_expectation(const std::string& code) {
    return compute_conditional_expectation_incremental(code);
}

void GaussianCopulaFactor::publish_prediction(const std::string& code, double prediction, int64_t timestamp) {
    auto& bus = DataBus::instance();
    bus.publish<double>(TOP_PREDICTION, code, timestamp, prediction);
}

} // namespace factorlib