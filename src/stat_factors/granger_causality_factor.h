#pragma once
/**
 * @file granger_causality_factor.h
 * @brief “格兰杰因果强度因子”：检验 OFI 是否格兰杰导致价格变化（输出 -log10 p 或 p）。
 */

#include <string>
#include <unordered_map>
#include <deque>
#include <vector>
#include <optional>
#include <cmath>
#include <limits>

#include "ifactor.h"                          // BaseFactor / IFactor
#include "utils/types.h"                      // QuoteDepth / Transaction / Entrust / Bar 等
#include "utils/log.h"
#include "utils/databus.h"                    // DataBus / safe_publish
#include "utils/nms_bucket_aggregator.h"      // NmsBucketAggregator
#include "utils/config/feed_mode.h"           // FeedMode 全局默认
#include "math/sliding_normal_eq.h"     // 增量法方程
#include "math/distributions.h"         // fisher_f_sf<T>()

namespace factorlib {

// =====================[ 配置 ]=====================
struct GrangerConfig {
    // --- 窗口与滞后 ---
    int     window_size   = 240;   ///< 回归滑窗长度 N（有效样本数上限）
    int     p_lags        = 2;     ///< y 的滞后阶 p
    int     q_lags        = 2;     ///< x(OFI) 的滞后阶 q
    int     min_effective = 16;    ///< 最小有效样本，低于则不发布

    // --- 喂数模式（默认走全局） ---
    config::FeedMode feed_mode = config::global_default_feed_mode();

    // --- 聚合参数（仅在 Aggregated 模式下使用） ---
    int64_t bucket_ms     = 1000;  ///< 聚合桶宽（毫秒）

    // --- 输出口径 ---
    bool    use_neglog10  = true;  ///< 输出 -log10(p)
    bool    publish_raw_p = false; ///< 额外发布 p 原值
    double  strength_clip = 20.0;  ///< -log10(p) 裁剪上限
};

// 主题名
static inline constexpr const char* TOP_GRANGER_STRENGTH = "granger/strength";
static inline constexpr const char* TOP_GRANGER_PVAL     = "granger/pval";

// =====================[ 因子类 ]=====================
class GrangerCausalityFactor : public BaseFactor {
public:
    explicit GrangerCausalityFactor(const std::vector<std::string>& codes,
                                    const GrangerConfig& cfg = {});

    // —— IFactor / BaseFactor 接口 —— //
    void on_quote(const QuoteDepth& q) override;
    // void on_transaction(const Transaction& t) override;
    // void on_entrust(const Entrust& e) override;
    void on_tick (const CombinedTick& x) override;   // 统一逐笔入口
    void on_bar(const Bar& b) override { (void)b; }
    bool force_flush(const std::string& code) override;

    // BaseFactor 的代码初始化钩子
    void on_code_added(const std::string& code) override;

    // 工程风格：静态注册 topic（不访问实例成员）
    static void register_topics(size_t capacity);

private:
    struct CodeState {
        std::optional<double> last_mid;  ///< 上一次中间价
        double pending_ofi {0.0};        ///< 逐条模式下累计 OFI
        std::deque<double> x_win;        ///< OFI 序列
        std::deque<double> y_win;        ///< dMid 序列

        int d_r {0}, d_u {0};
        math::SlidingNormalEq<double> ne_r {1, 240};
        math::SlidingNormalEq<double> ne_u {1, 240};

        NmsBucketAggregator bucket {1000};
        int64_t last_bucket_ts {0};
    };

    void on_any_event_(const std::string& code, int64_t ts_ms,
                       const std::optional<QuoteDepth>& qopt,
                       const std::optional<Transaction>& topt,
                       const std::optional<Entrust>& eopt);

    void ensure_code_(const std::string& code);
    void close_bucket_and_push_(const std::string& code, const BucketOutputs& bkt);
    void emit_sample_event_driven_(const std::string& code, int64_t ts_ms, double y_t);

    void push_sample_and_update_(const std::string& code, int64_t ts_ms,
                                 double y_t,
                                 const std::vector<double>& y_lags,
                                 const std::vector<double>& x_lags);

    void publish_strength_(const std::string& code, int64_t ts_ms, double p) const;

private:
    GrangerConfig _cfg;
    std::unordered_map<std::string, CodeState> _states;
};

} // namespace factorlib
