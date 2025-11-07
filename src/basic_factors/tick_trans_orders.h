
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
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

namespace factorlib {

/// 配置项：只包含桶宽度（毫秒）
struct TickTransOrdersConfig { int64_t bucket_size_ms = 1000; };

/**
 * @brief 0109 因子的“翻译”实现（基础因子，写入 DataBus）
 * 使用：
 *   - 先调用静态方法 register_topics(cap) 注册五个 topic；
 *   - 实例化本类，依次喂入 on_quote/on_transaction/on_entrust；
 *   - 若当前时间跨桶，自动在下一次 on_* 调用时产出并发布上一桶；
 *   - 可在收盘或测试时调用 force_flush 手动产出。
 */
class TickTransOrders {
public:
    explicit TickTransOrders(const TickTransOrdersConfig& cfg, std::vector<std::string> codes)
        : _cfg(cfg), _codes(std::move(codes)) {}

    /// 注册五个 topic，容量默认 120（可配置）
    static void register_topics(size_t capacity_per_topic=120);

    /// 喂入一条行情（用于计算增量与中价）
    void on_quote(const QuoteDepth& q);
    /// 喂入一条成交（用于桶内切片）
    void on_transaction(const Transaction& t);
    /// 喂入一条委托（用于桶内切片）
    void on_entrust(const Entrust& e);

    /// 强制产出某代码的当前桶（返回是否产出成功）
    bool force_flush(const std::string& code);

private:
    TickTransOrdersConfig _cfg;
    std::vector<std::string> _codes;
    std::unordered_map<std::string, NmsBucketAggregator> _agg;

    void ensure_code(const std::string& code);
    void maybe_flush_and_publish(const std::string& code, int64_t now_ms);
    void publish_bucket(const std::string& code, const BucketOutputs& out);
};

} // namespace factorlib
