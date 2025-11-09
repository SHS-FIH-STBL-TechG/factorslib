#pragma once
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include "utils/utils.h"
#include "utils/databus.h"
#include "utils/log.h"

/**
 * @file gaussian_copula_factor.h
 * @brief 高斯 Copula 条件期望因子
 * 
 * 计算步骤：
 * 1. 使用订单流不平衡(OFI)与成交量(V)构成二维特征
 * 2. 与下一期的对数收益率一并整合为秩
 * 3. 求中位秩分位，转换为正态分位数
 * 4. 计算样本协方差矩阵
 * 5. 求条件高斯下的条件均值
 * 6. 使用经验逆CDF把概率变成下一期收益的数值预测
 */

namespace factorlib {

/// 高斯 Copula 因子配置
struct GaussianCopulaConfig {
    int window_size = 30;           ///< 滑动窗口大小
    double regularization = 1e-6;   ///< 协方差矩阵正则化参数
    bool debug_mode = false;        ///< 是否输出调试信息
};

/**
 * @brief 高斯 Copula 条件期望因子
 * 
 * 输入：订单流数据 (OFI, 成交量)
 * 输出：下一期收益率的条件期望预测
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
    
private:
    GaussianCopulaConfig _cfg;
    std::vector<std::string> _codes;
    
    // 每个代码的状态
    struct CodeState {
        std::deque<double> ofi_window;      ///< OFI滑动窗口
        std::deque<double> volume_window;   ///< 成交量滑动窗口  
        std::deque<double> return_window;   ///< 收益率滑动窗口
        double last_mid_price = 0.0;        ///< 上一期中间价
        double current_ofi = 0.0;           ///< 当前tick累计OFI
        double current_volume = 0.0;        ///< 当前tick累计成交量
        bool has_initial_price = false;     ///< 是否有初始价格
    };
    
    std::unordered_map<std::string, CodeState> _states;
    
    /// 确保代码状态存在
    void ensure_code(const std::string& code);
    
    /// 计算中位秩分位
    double compute_median_rank(const std::deque<double>& data, double value);
    
    /// 正态分布逆CDF（使用近似公式）
    double normal_quantile(double p);
    
    /// 计算经验逆CDF
    double empirical_inverse_cdf(const std::deque<double>& returns, double probability);
    
    /// 计算条件期望
    double compute_conditional_expectation(const std::string& code);
    
    /// 发布因子值
    void publish_prediction(const std::string& code, double prediction, int64_t timestamp);
};

} // namespace factorlib