//  src/basic_factors/tick_trans_orders.h
#pragma once
#include <string>
#include <vector>
#include <unordered_map>

#include "ifactor.h"
#include "utils/utils.h"
#include "utils/databus.h"
#include "utils/log.h"

/**
 * @file tick_trans_orders.h
 * @brief 示例“基础因子”：把 tick / 成交 / 委托 按 N 毫秒聚合到时间桶，
 *        并发布 0109 对应的五类 topic：
 *          - zyd/amount, zyd/volume, zyd/midprice
 *          - zyd/tick/trans, zyd/tick/orders
 */


// 对外暴露 interval 主题常量（供测试/其他模块使用）
static inline constexpr const char* TOP_INTERVAL_TRANS  = "zyd/interval/trans";
static inline constexpr const char* TOP_INTERVAL_ORDERS = "zyd/interval/orders";
namespace factorlib {

/// 配置项：只包含桶宽度（毫秒）
struct TickTransOrdersConfig {
    int64_t bucket_size_ms = 1000;
    bool    emit_tick_interval = false; ///< 是否发布两个 tick 之间的切片
};

/**
 * @brief 0109 因子的“翻译”实现（基础因子，写入 DataBus）
 * 使用：
 *   - 先调用静态方法 register_topics(cap) 注册五个 topic；
 *   - 实例化本类，依次喂入 on_quote/on_transaction/on_entrust；
 *   - 若当前时间跨桶，自动在下一次 on_* 调用时产出并发布上一桶；
 *   - 可在收盘或测试时调用 force_flush 手动产出。
 */
class TickTransOrders : public BaseFactor{
public:
    explicit TickTransOrders(const TickTransOrdersConfig& cfg, std::vector<std::string> codes)
        : BaseFactor("TickTransOrders", std::move(codes)), _cfg(cfg) {}

    /// 注册五个 topic，容量默认 120（可配置）
    static void register_topics(size_t capacity=120);

    /// 喂入一条行情（用于计算增量与中价）
    void on_quote(const QuoteDepth& q) override;
    /// 喂入一条成交（用于桶内切片）
    void on_transaction(const Transaction& t) override;
    /// 喂入一条委托（用于桶内切片）
    void on_entrust(const Entrust& e) override;

    /// 强制产出某代码的当前桶（返回是否产出成功）
    bool force_flush(const std::string& code) override;

private:
    TickTransOrdersConfig _cfg;
    std::vector<std::string> _codes;
    std::unordered_map<std::string, NmsBucketAggregator> _agg;
    // --- interval 相关：在相邻 tick 之间暂存切片，下一笔 tick 到来时发布 ---
    std::unordered_map<std::string, int64_t> _last_tick_ms;
    std::unordered_map<std::string, std::vector<Transaction>> _interval_trans_pending;
    std::unordered_map<std::string, std::vector<Entrust>> _interval_orders_pending;

    void ensure_code(const std::string& code);
    void maybe_flush_and_publish(const std::string& code, int64_t now_ms);
    void publish_bucket(const std::string& code, const BucketOutputs& out);
};

} // namespace factorlib
