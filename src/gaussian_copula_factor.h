// src/gaussian_copula_factor.h
#pragma once
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <memory>
#include "utils/types.h"        // 替换 utils/utils.h
#include "utils/databus.h"
#include "utils/log.h"
#include "utils/math/incremental_rank.h"

/**
 * @file gaussian_copula_factor.h
 * @brief 高斯 Copula 条件期望因子（增量计算优化版）
 */

namespace factorlib {

/// 高斯 Copula 因子配置
struct GaussianCopulaConfig {
    int window_size = 30;           ///< 滑动窗口大小
    double regularization = 1e-6;   ///< 协方差矩阵正则化参数
    bool debug_mode = false;        ///< 是否输出调试信息
};

/**
 * @brief 高斯 Copula 条件期望因子（增量计算优化版）
 *
 * 使用增量秩计算器和增量协方差计算器，实现 O(log n) 时间复杂度的滑动窗口计算
 */
class GaussianCopulaFactor {
public:
    explicit GaussianCopulaFactor(const GaussianCopulaConfig& cfg, std::vector<std::string> codes);

    /// 注册因子输出主题
    static void register_topics(size_t capacity = 120);

    /// 处理行情数据（用于计算收益率）
    void on_quote(const QuoteDepth& q);

    /// 处理委托数据（用于计算OFI）
    void on_entrust(const Entrust& e);

    /// 处理成交数据（可选，用于验证）
    void on_transaction(const Transaction& t);

    /// 强制刷新当前计算
    bool force_flush(const std::string& code);

    /// 获取因子名称
    std::string get_name() const { return "GaussianCopulaFactor"; }

    /// 获取监控的股票代码列表
    const std::vector<std::string>& get_codes() const { return _codes; }

private:
    GaussianCopulaConfig _cfg;
    std::vector<std::string> _codes;

    // 每个代码的原始状态
    struct CodeState {
        std::deque<double> ofi_window;      ///< OFI滑动窗口（保留用于兼容性）
        std::deque<double> volume_window;   ///< 成交量滑动窗口（保留用于兼容性）
        std::deque<double> return_window;   ///< 收益率滑动窗口（保留用于兼容性）
        double last_mid_price = 0.0;        ///< 上一期中间价
        double current_ofi = 0.0;           ///< 当前tick累计OFI
        double current_volume = 0.0;        ///< 当前tick累计成交量
        bool has_initial_price = false;     ///< 是否有初始价格
    };

    std::unordered_map<std::string, CodeState> _states;

    // 增量计算状态
    struct IncrementalState {
        size_t window_size;
        math::IncrementalRankCalculator<double> ofi_rank_calc;
        math::IncrementalRankCalculator<double> volume_rank_calc;
        math::IncrementalRankCalculator<double> return_rank_calc;
        math::IncrementalCovariance<double, 3> cov_calc;

        explicit IncrementalState(size_t window_size);

        void update_data(double ofi, double volume, double ret);
        bool is_window_full() const;
    };

    std::unordered_map<std::string, std::unique_ptr<IncrementalState>> _incremental_states;

    /// 确保代码状态存在
    void ensure_code(const std::string& code);

    /// 增量计算条件期望
    double compute_conditional_expectation_incremental(const std::string& code);

    /// 全量计算条件期望（保留用于兼容性）
    double compute_conditional_expectation(const std::string& code);
    
    /// 发布因子值
    void publish_prediction(const std::string& code, double prediction, int64_t timestamp);
};

} // namespace factorlib