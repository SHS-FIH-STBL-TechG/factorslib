#include "gaussian_copula_factor.h"
#include <Eigen/Dense>  // 需要添加Eigen库依赖

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
}

void GaussianCopulaFactor::on_quote(const QuoteDepth& q) {
    ensure_code(q.instrument_id);
    auto& state = _states[q.instrument_id];
    
    // 计算当前中间价
    double mid_price = (q.bid_price + q.ask_price) / 2.0;
    
    if (state.has_initial_price) {
        // 计算对数收益率: log(当前价/上一期价)
        double log_return = std::log(mid_price / state.last_mid_price);
        
        // 将上一期的OFI、成交量和当前收益率加入窗口
        if (state.ofi_window.size() >= static_cast<size_t>(_cfg.window_size)) {
            state.ofi_window.pop_front();
            state.volume_window.pop_front();
            state.return_window.pop_front();
        }
        
        state.ofi_window.push_back(state.current_ofi);
        state.volume_window.push_back(state.current_volume);
        state.return_window.push_back(log_return);
        
        // 重置当前tick的累计值
        state.current_ofi = 0.0;
        state.current_volume = 0.0;
        
        // 如果窗口已满，计算因子值
        if (state.ofi_window.size() >= static_cast<size_t>(_cfg.window_size)) {
            double prediction = compute_conditional_expectation(q.instrument_id);
            publish_prediction(q.instrument_id, prediction, q.data_time_ms);
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
}

void GaussianCopulaFactor::on_transaction(const Transaction& t) {
    // 成交数据可用于验证，但主要计算依赖委托数据
    // 这里可以选择性地实现一些验证逻辑
}

bool GaussianCopulaFactor::force_flush(const std::string& code) {
    auto it = _states.find(code);
    if (it == _states.end() || it->second.ofi_window.size() < static_cast<size_t>(_cfg.window_size)) {
        return false;
    }
    
    double prediction = compute_conditional_expectation(code);
    publish_prediction(code, prediction, 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    return true;
}

double GaussianCopulaFactor::compute_median_rank(const std::deque<double>& data, double value) {
    if (data.empty()) return 0.5;
    
    // 创建数据副本并排序
    std::vector<double> sorted_data(data.begin(), data.end());
    std::sort(sorted_data.begin(), sorted_data.end());
    
    // 找到值的排名
    auto it = std::lower_bound(sorted_data.begin(), sorted_data.end(), value);
    size_t rank = std::distance(sorted_data.begin(), it);
    
    // 计算中位秩: (rank + 1 - 0.5) / N
    return (rank + 0.5) / sorted_data.size();
}

double GaussianCopulaFactor::normal_quantile(double p) {
    // 使用Wichura算法的近似实现
    // 这是一个简化的版本，实际应用中可以使用更精确的实现
    if (p <= 0 || p >= 1) return 0.0;
    
    static const double a1 = -3.969683028665376e+01;
    static const double a2 =  2.209460984245205e+02;
    static const double a3 = -2.759285104469687e+02;
    static const double a4 =  1.383577518672690e+02;
    static const double a5 = -3.066479806614716e+01;
    static const double a6 =  2.506628277459239e+00;
    
    static const double b1 = -5.447609879822406e+01;
    static const double b2 =  1.615858368580409e+02;
    static const double b3 = -1.556989798598866e+02;
    static const double b4 =  6.680131188771972e+01;
    static const double b5 = -1.328068155288572e+01;
    
    static const double c1 = -7.784894002430293e-03;
    static const double c2 = -3.223964580411365e-01;
    static const double c3 = -2.400758277161838e+00;
    static const double c4 = -2.549732539343734e+00;
    static const double c5 =  4.374664141464968e+00;
    static const double c6 =  2.938163982698783e+00;
    
    static const double d1 =  7.784695709041462e-03;
    static const double d2 =  3.224671290700398e-01;
    static const double d3 =  2.445134137142996e+00;
    static const double d4 =  3.754408661907416e+00;
    
    double q, r;
    
    if (p < 0.02425) {
        q = std::sqrt(-2.0 * std::log(p));
        return (((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
               ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
    } else if (p > 0.97575) {
        q = std::sqrt(-2.0 * std::log(1.0 - p));
        return -(((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
                ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
    } else {
        q = p - 0.5;
        r = q * q;
        return (((((a1 * r + a2) * r + a3) * r + a4) * r + a5) * r + a6) * q /
               (((((b1 * r + b2) * r + b3) * r + b4) * r + b5) * r + 1.0);
    }
}

double GaussianCopulaFactor::empirical_inverse_cdf(const std::deque<double>& returns, double probability) {
    if (returns.empty()) return 0.0;
    
    // 创建排序副本
    std::vector<double> sorted_returns(returns.begin(), returns.end());
    std::sort(sorted_returns.begin(), sorted_returns.end());
    
    // 计算分位数位置
    double position = probability * (sorted_returns.size() - 1);
    size_t index = static_cast<size_t>(std::floor(position));
    double fraction = position - index;
    
    // 线性插值
    if (index == sorted_returns.size() - 1) {
        return sorted_returns.back();
    } else {
        return sorted_returns[index] + fraction * (sorted_returns[index + 1] - sorted_returns[index]);
    }
}

double GaussianCopulaFactor::compute_conditional_expectation(const std::string& code) {
    auto& state = _states[code];
    size_t n = state.ofi_window.size();
    
    if (n < 3) return 0.0;  // 需要足够的数据点
    
    // 步骤1: 秩转换和正态分位数转换
    std::vector<double> z_ofi(n), z_volume(n), z_return(n);
    
    for (size_t i = 0; i < n; ++i) {
        double rank_ofi = compute_median_rank(state.ofi_window, state.ofi_window[i]);
        double rank_volume = compute_median_rank(state.volume_window, state.volume_window[i]);
        double rank_return = compute_median_rank(state.return_window, state.return_window[i]);
        
        z_ofi[i] = normal_quantile(rank_ofi);
        z_volume[i] = normal_quantile(rank_volume);
        z_return[i] = normal_quantile(rank_return);
    }
    
    // 步骤2: 计算协方差矩阵
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    
    // 计算均值
    for (size_t i = 0; i < n; ++i) {
        mean(0) += z_ofi[i];
        mean(1) += z_volume[i];
        mean(2) += z_return[i];
    }
    mean /= n;
    
    // 计算协方差
    for (size_t i = 0; i < n; ++i) {
        Eigen::Vector3d dev;
        dev << z_ofi[i] - mean(0), z_volume[i] - mean(1), z_return[i] - mean(2);
        covariance += dev * dev.transpose();
    }
    covariance /= (n - 1);
    
    // 正则化协方差矩阵
    covariance += Eigen::Matrix3d::Identity() * _cfg.regularization;
    
    // 步骤3: 计算当前观测值的Z分数
    double current_ofi_rank = compute_median_rank(state.ofi_window, state.current_ofi);
    double current_volume_rank = compute_median_rank(state.volume_window, state.current_volume);
    
    double z_ofi_current = normal_quantile(current_ofi_rank);
    double z_volume_current = normal_quantile(current_volume_rank);
    
    // 步骤4: 计算条件期望
    // 分割协方差矩阵:
    // Σ = [[Σ_11, Σ_12],  其中 Σ_11 是 [OFI, Volume] 的协方差
    //      [Σ_21, Σ_22]]  Σ_22 是 Return 的方差
    //                    Σ_12 = Σ_21^T 是交叉协方差
    
    Eigen::Matrix2d sigma_11 = covariance.block<2,2>(0,0);
    Eigen::Vector2d sigma_12 = covariance.block<2,1>(0,2);
    double sigma_22 = covariance(2,2);
    
    // 条件均值公式: μ_2|1 = Σ_21 * Σ_11^{-1} * (x_1 - μ_1)
    Eigen::Vector2d x_condition;
    x_condition << z_ofi_current - mean(0), z_volume_current - mean(1);
    
    Eigen::Vector2d beta = sigma_11.ldlt().solve(sigma_12);
    double conditional_mean = mean(2) + beta.dot(x_condition);
    
    // 步骤5: 转换回均匀分布并应用经验逆CDF
    double conditional_probability = 0.5 * (1.0 + std::erf(conditional_mean / std::sqrt(2.0)));
    double predicted_return = empirical_inverse_cdf(state.return_window, conditional_probability);
    
    if (_cfg.debug_mode) {
        SPDLOG_DEBUG("GaussianCopulaFactor[{}]: OFI={}, Volume={}, PredictedReturn={}", 
                    code, state.current_ofi, state.current_volume, predicted_return);
    }
    
    return predicted_return;
}

void GaussianCopulaFactor::publish_prediction(const std::string& code, double prediction, int64_t timestamp) {
    auto& bus = DataBus::instance();
    bus.publish<double>(TOP_PREDICTION, code, timestamp, prediction);
}

} // namespace factorlib