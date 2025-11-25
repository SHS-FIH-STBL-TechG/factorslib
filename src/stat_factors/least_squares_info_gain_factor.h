#pragma once
/**
 * @file least_squares_info_gain_factor.h
 * @brief 最小二乘信息增益因子：
 *        y_t = r_t, x_t = r_{t-1}，滑动 OLS 计算 ln(Var(y)/Var(res)) 及其 95% 置信区间。
 */

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

#include "ifactor.h"
#include "utils/types.h"
#include "math/sliding_normal_eq.h"
#include "math/sliding_statistics.h"

namespace factorlib {

// ------------ 配置 ------------

struct LsInfoGainConfig {
    /// 滑动窗口长度（样本条数）。窗口满窗后，每来一条新价格事件就更新一次因子。
    int  window_size = 30;
};

// 主题名
inline constexpr const char* TOP_LSIG_IG    = "lsig/ig";     // 主因子：ln(Var原始 / Var残差)
inline constexpr const char* TOP_LSIG_IG_LO = "lsig/ig_lo";  // 95% CI 下界
inline constexpr const char* TOP_LSIG_IG_HI = "lsig/ig_hi";  // 95% CI 上界

    // ------------ 因子类 ------------

    class LeastSquaresInfoGainFactor : public BaseFactor {
    public:
        LeastSquaresInfoGainFactor(const std::vector<std::string>& codes,
                                   const LsInfoGainConfig& cfg = {});

        /// 注册 DataBus topic
        static void register_topics(size_t capacity = 1024);

        // IFactor 接口：三类数据都能驱动
        void on_quote(const QuoteDepth& q) override;          // 快照
        void on_tick(const CombinedTick& x) override;         // 逐笔成交 / 委托（都走 CombinedTick）
        void on_bar(const Bar& b) override;                   // K 线

        bool force_flush(const std::string& code) override {
            (void)code;
            return false;
        }

        void on_code_added(const std::string& code) override;

    private:
        using SlidingOLS   = math::SlidingNormalEq<double>;
        using SlidingStats = math::SlidingWindowStats<double>;

        struct CodeState {
            bool   has_last_px{false};
            double last_px{0.0};

            bool   has_prev_ret{false};
            double prev_ret{0.0};

            int    window_size{0};
            std::unique_ptr<SlidingOLS>   ols;
            std::unique_ptr<SlidingStats> y_stats;
        };

        LsInfoGainConfig _cfg;
        std::vector<int> _window_sizes; ///< 支持一次性运行多个窗口长度
        std::unordered_map<std::string, CodeState> _states;

        CodeState& ensure_state(const ScopeKey& scope);

        /// 统一价格事件入口：任何来源（快照/逐笔/K 线）的价格都走这里
        void on_price_event(const std::string& code_raw, int64_t ts_ms, double px);

        void push_sample_and_update(CodeState& S,
                                    const ScopeKey& scope,
                                    int64_t ts_ms,
                                    double x_prev_ret,
                                    double y_curr_ret);

        void publish_all(const std::string& scoped_code,
                         int64_t ts_ms,
                         double ig,
                         double ig_lo,
                         double ig_hi);
    };


} // namespace factorlib
