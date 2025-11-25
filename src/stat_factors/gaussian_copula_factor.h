// src/gaussian_copula_factor.h
#pragma once
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>

#include "ifactor.h"                 // ★ 新增：继承 BaseFactor/IFactor
#include "utils/types.h"
#include "utils/databus.h"
#include "utils/log.h"
#include "math/incremental_rank.h"

/**
 * @file gaussian_copula_factor.h
 * @brief 高斯 Copula 条件期望因子（增量计算优化版）
 *
 * ★ 本版仅做“继承 BaseFactor”的集成性改造，不改你的计算/窗口/发布语义。
 */

namespace factorlib {

/// 高斯 Copula 因子配置
struct GaussianCopulaConfig {
    int window_size = 30;           ///< 滑动窗口大小
    double regularization = 1e-6;   ///< 协方差矩阵正则化参数
};

/**
 * @brief 高斯 Copula 条件期望因子（增量计算优化版）
 *
 * 使用增量秩计算器和增量协方差计算器，实现 O(log n) 的滑动窗口计算
 * ★ 改造点：继承 BaseFactor，使其可被 bridge::ingest_* 统一驱动
 */
class GaussianCopulaFactor : public BaseFactor {
public:
    /**
     * @brief 构造函数
     * @param cfg   配置
     * @param codes 关注代码（委托给 BaseFactor 管理）
     *
     * ★ 行为不变：仍然使用秩/协方差窗口；仅在窗口满足时发布
     */
    explicit GaussianCopulaFactor(const GaussianCopulaConfig& cfg, std::vector<std::string> codes);

    /// 注册因子输出主题（主题名与 demo 对齐）
    static void register_topics(size_t capacity = 120);

    // ===== IFactor 接口：由 bridge 驱动 =====
    void on_quote(const QuoteDepth& q) override;          ///< 行情：用于计算对数收益
    // void on_entrust(const Entrust& e) override;           ///< 委托：累计 OFI/Volume
    // void on_transaction(const Transaction& t) override;   ///< 成交：可选（原逻辑未使用）
    void on_tick (const CombinedTick& x) override;   // 统一逐笔入口
    void on_bar(const Bar& /*b*/) override {}             ///< 当前未用
    bool force_flush(const std::string& code) override;   ///< 仅窗口满时返回 true 并发布

    // ===== 为兼容旧调用，保留同名方法（委托给 BaseFactor）=====
    std::string get_name() const { return BaseFactor::get_name(); }
    const std::vector<std::string>& get_codes() const { return BaseFactor::get_codes(); }

protected:
    /**
     * @brief BaseFactor 钩子：首次遇到某代码时调用
     * ★ 这里不做初始化（避免与 ensure_code 重复初始化），保持原行为：
     *    各回调一律先调用 ensure_code(code) 完成初始化。
     */
    void on_code_added(const std::string& /*code*/) override {}

private:
    GaussianCopulaConfig _cfg;
    std::vector<int> _window_sizes; ///< 支持多窗口同时运行，列表来源于配置

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

        /// 新增数据点并维护秩与协方差；当三个秩都达到窗口大小时才推进协方差
        void update_data(double ofi, double volume, double ret);
        /// 窗口是否已满：三个秩与协方差都达到窗口大小
        bool is_window_full() const;
    };
    std::unordered_map<std::string, std::unique_ptr<IncrementalState>> _incremental_states;

    CodeState& ensure_state(const ScopeKey& scope);
    IncrementalState& ensure_incremental(const ScopeKey& scope);

    /// 增量计算条件期望（★ 不改你的算法）
    double compute_conditional_expectation_incremental(const std::string& scoped_code);

    /// 全量计算条件期望（保留用于兼容性）
    double compute_conditional_expectation(const std::string& scoped_code);

    /// 发布因子值（主题固定）
    void publish_prediction(const std::string& scoped_code, double prediction, int64_t timestamp);
};

} // namespace factorlib
