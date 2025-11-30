// src/factors/basic/tick_trans_orders.cpp
#include "factors/basic/tick_trans_orders.h"
#include "utils/log.h"
#include "core/databus.h"  // safe_publish 封装

/**
 * @file tick_trans_orders.cpp
 *
 * 功能概述（对应你旧版的大因子）：
 *  1. 使用 NmsBucketAggregator，把 Quote / 逐笔（成交+委托）聚合到固定毫秒的时间桶中；
 *     - 对每个桶产出 5 类指标：
 *         amount_sum     -> zyd/amount
 *         volume_sum     -> zyd/volume
 *         midprice_last  -> zyd/midprice
 *         trans（桶内所有成交）  -> zyd/tick/trans
 *         orders（桶内所有委托） -> zyd/tick/orders
 *
 *  2. 可选（通过 _cfg.emit_tick_interval 开关）：
 *     - 在 “两个 Quote tick 之间” 收集所有成交 / 委托：
 *         interval/trans  -> zyd/interval/trans
 *         interval/orders -> zyd/interval/orders
 *       语义上等价于旧版的 _RingTickTrans / _RingTickOrders。
 *
 *  3. 在 emit_tick_interval 开启时，额外产出 “tick 维度” 的 3 个指标：
 *     - tick 间成交额：  zyd/tick/amount
 *     - tick 间成交量：  zyd/tick/volume
 *     - tick midprice： zyd/tick/midprice（使用当前 Quote 的 bid/ask 中间价）
 *
 *  这样，新版产出的信息量 ≥ 旧版：
 *    - 旧版有的 tick 级 amount/volume/midprice + tick 间成交/委托，新版都有；
 *    - 新版多出来一套 Nms 桶聚合的 5 个时间序列。
 */

namespace factorlib {

    // ====================== topic 常量定义 ======================

    // ---- 桶级（Nms 聚合） topic ----
    // 注意这里的 "tick" 在命名上沿用旧习惯，实际粒度是“时间桶”
    static const char* TOP_AMOUNT   = "zyd/amount";       // 每个时间桶内的成交额总和
    static const char* TOP_VOLUME   = "zyd/volume";       // 每个时间桶内的成交量总和
    static const char* TOP_MID      = "zyd/midprice";     // 每个时间桶最后一个 midprice
    static const char* TOP_TTRANS   = "zyd/tick/trans";   // 每个时间桶内所有成交集合
    static const char* TOP_TORD     = "zyd/tick/orders";  // 每个时间桶内所有委托集合

    // ---- tick 间切片 topic（两个 Quote tick 之间的成交 / 委托）----
    static const char* TOP_IVTRANS  = "zyd/interval/trans";   // tick 间成交切片
    static const char* TOP_IVORD    = "zyd/interval/orders";  // tick 间委托切片

    // ---- tick 级指标 topic（对标旧版 per-tick 输出）----
    static const char* TOP_TICK_AMOUNT  = "zyd/tick/amount";    // 单个 tick 间隔内的成交额
    static const char* TOP_TICK_VOLUME  = "zyd/tick/volume";    // 单个 tick 间隔内的成交量
    static const char* TOP_TICK_MID     = "zyd/tick/midprice";  // 单个 tick 的 midprice

    // ====================== TickTransOrders 成员实现 ======================

    /**
     * @brief 在 DataBus 上注册所有需要用到的 topic
     *
     * @param cap 每个 topic 的容量（环形队列长度）
     *
     * 说明：
     *  - register_topics 一般在框架初始化时调用一次；
     *  - 这里只注册 topic，不做任何逻辑。
     */
    void TickTransOrders::register_topics(size_t cap){
        auto& bus = DataBus::instance();

        // 1）Nms 时间桶的 5 个输出
        bus.register_topic<double>(TOP_AMOUNT, cap);
        bus.register_topic<int64_t>(TOP_VOLUME, cap);
        bus.register_topic<double>(TOP_MID, cap);
        bus.register_topic<std::vector<Transaction>>(TOP_TTRANS, cap);
        bus.register_topic<std::vector<Entrust>>(TOP_TORD, cap);

        // 2）两个 tick 之间的成交 / 委托切片
        bus.register_topic<std::vector<Transaction>>(TOP_IVTRANS, cap);
        bus.register_topic<std::vector<Entrust>>(TOP_IVORD, cap);

        // 3）tick 维度的 3 个指标（amount / volume / midprice）
        bus.register_topic<double>(TOP_TICK_AMOUNT, cap);
        bus.register_topic<int64_t>(TOP_TICK_VOLUME, cap);
        bus.register_topic<double>(TOP_TICK_MID, cap);
    }

    /**
     * @brief 确保某个代码已经在内部状态中初始化
     *
     * 负责：
     *  - 为该代码创建 NmsBucketAggregator；
     *  - 初始化 last_tick_ms；
     *  - 初始化 interval pending 缓冲；
     *  - 同步桶宽（bucket_size_ms）。
     */
    void TickTransOrders::ensure_code(const std::string& code){
        if (_agg.find(code) == _agg.end()){
            // 为新代码创建一个聚合器，初始桶宽为 _cfg.bucket_size_ms
            _agg.emplace(code, NmsBucketAggregator(_cfg.bucket_size_ms));

            // 上一个 quote tick 的时间戳（毫秒）
            _last_tick_ms[code] = 0;

            // tick 间成交 / 委托 pending 容器
            _interval_trans_pending[code] = {};
            _interval_orders_pending[code] = {};
        }
        // 如果外部配置有更新，确保桶宽一致
        _agg[code].set_bucket_ms(_cfg.bucket_size_ms);
    }

    /**
     * @brief 把聚合器输出的一个 BucketOutputs 发布到 DataBus
     *
     * @param code 标的代码
     * @param out  单个时间桶的聚合结果
     *
     * 说明：
     *  - 时间戳统一使用 bucket_end_ms 作为该桶的时间；
     *  - trans / orders 是整个桶内的成交 / 委托集合。
     */
    void TickTransOrders::publish_bucket(const std::string& code, const BucketOutputs& out){
        auto& bus = DataBus::instance();
        const int64_t ts = out.bucket_end_ms;  // 统一使用桶结束时间作为时间戳

        safe_publish<double>(TOP_AMOUNT,  code, ts, out.amount_sum);
        safe_publish<int64_t>(TOP_VOLUME, code, ts, out.volume_sum);
        safe_publish<double>(TOP_MID,     code, ts, out.midprice_last);
        safe_publish<std::vector<Transaction>>(TOP_TTRANS, code, ts, out.trans);
        safe_publish<std::vector<Entrust>>(   TOP_TORD,   code, ts, out.orders);
    }

    /**
     * @brief 检查当前时间是否跨桶，并在需要时发布桶结果
     *
     * @param code   标的代码
     * @param now_ms 当前时间（毫秒）
     *
     * 说明：
     *  - ensure_bucket(now_ms, out) 一般用于“保证当前时间点之前的桶都已经生成”；
     *  - flush_if_crossed(now_ms, out) 用于按时间推进做 flush；
     *  - 具体行为由 NmsBucketAggregator 内部定义，这里只负责调用和转发结果。
     */
    void TickTransOrders::maybe_flush_and_publish(const std::string& code, int64_t now_ms) {
        _agg[code].ensure_bucket(now_ms);
        BucketOutputs out;
        while (_agg[code].pop_ready(out)) {
            publish_bucket(code, out);
        }
        while (_agg[code].flush_if_crossed(now_ms, out)) {
            publish_bucket(code, out);
        }
    }

    /**
     * @brief 处理一条 L2 行情（QuoteDepth）
     *
     * 主要逻辑分三部分：
     *   1）如果启用了 emit_tick_interval：
     *      - 以“quote 到来”作为 tick 边界：
     *          * 把上一 tick 以来缓存的成交 / 委托视为一个 tick 间隔；
     *          * 对该间隔计算 tick amount / volume / midprice；
     *          * 发布到 tick 维度的 3 个 topic；
     *          * 同时把该间隔的成交 / 委托切片发布到 interval topic；
     *          * 清空 pending，更新 last_tick_ms。
     *   2）调用 maybe_flush_and_publish：确保 Nms 时间桶按时间推进；
     *   3）把当前 quote 喂给 NmsBucketAggregator，再次检查是否有桶完成需要发布。
     */
    void TickTransOrders::on_quote(const QuoteDepth& q) {
        ensure_code(q.instrument_id);

        // -------- 1. tick 间切片 + tick 级指标 --------
        if (_cfg.emit_tick_interval) {
            auto &lt = _last_tick_ms[q.instrument_id];
            if (lt > 0) {
                // 1.1 上一个 tick 以来缓存的成交 / 委托
                auto &vt = _interval_trans_pending[q.instrument_id];  // interval 内的所有 Transaction
                auto &vo = _interval_orders_pending[q.instrument_id]; // interval 内的所有 Entrust

                // 1.2 用 vt 计算 “当前 tick 间隔” 的 amount / volume
                double  tick_amount = 0.0;
                int64_t tick_volume = 0;

                for (const auto& t : vt) {
                    // 成交额 = price * volume
                    // 如需要考虑合约乘数，可以在这里对 volume 做单位换算
                    tick_amount += t.price * static_cast<double>(t.volume);
                    tick_volume += static_cast<int64_t>(t.volume);
                }

                // 1.3 计算 tick 级 midprice
                //     这里直接使用当前 quote 的 top-of-book：
                //        mid = (bid_price + ask_price) / 2
                //     若盘口无效（bid 或 ask 为 0），则设置为 0.0（不引入额外状态）。
                double tick_mid = 0.0;
                if (q.bid_price > 0.0 && q.ask_price > 0.0) {
                    tick_mid = (q.bid_price + q.ask_price) * 0.5;
                } else {
                    tick_mid = 0.0;
                }

                // 1.4 发布 tick 维度的 3 个指标
                //     时间戳选择当前 quote 的时间（视为“本 tick 的结束时刻”）
                safe_publish<double>(
                    TOP_TICK_AMOUNT,
                    q.instrument_id,
                    q.data_time_ms,
                    tick_amount
                );
                safe_publish<int64_t>(
                    TOP_TICK_VOLUME,
                    q.instrument_id,
                    q.data_time_ms,
                    tick_volume
                );
                safe_publish<double>(
                    TOP_TICK_MID,
                    q.instrument_id,
                    q.data_time_ms,
                    tick_mid
                );

                // 1.5 同时发布 “两个 tick 之间的成交 / 委托切片”
                //     语义上等价旧版的 _RingTickTrans / _RingTickOrders
                safe_publish<std::vector<Transaction>>(
                    TOP_IVTRANS,
                    q.instrument_id,
                    q.data_time_ms,
                    vt
                );
                safe_publish<std::vector<Entrust>>(
                    TOP_IVORD,
                    q.instrument_id,
                    q.data_time_ms,
                    vo
                );

                // 1.6 清空本次 interval 缓存，等待下一个 tick
                vt.clear();
                vo.clear();
            }

            // 更新“上一个 tick 时间”，下次成交/委托用来判断是否属于新 tick 间隔
            _last_tick_ms[q.instrument_id] = q.data_time_ms;
        }

        // -------- 2. Nms 时间桶聚合逻辑（完全保持原样） --------
        // 在调用聚合器之前先检查桶边界：确保在当前 quote 时间之前的桶都已经处理
        maybe_flush_and_publish(q.instrument_id, q.data_time_ms);

        // 把当前 quote 喂给聚合器
        _agg[q.instrument_id].on_quote(q);

        // Quote 可能触发桶滚动，再检查一次
        maybe_flush_and_publish(q.instrument_id, q.data_time_ms);
    }

    /**
     * @brief 处理一条合表逐笔（成交或委托）
     *
     * 输入 CombinedTick：
     *  - kind = Trade：视为成交 Transaction；
     *  - kind = Order：视为委托 Entrust。
     *
     * 主要逻辑：
     *  1）转换为 Transaction / Entrust 的最小字段结构；
     *  2）如果 emit_tick_interval 开启：
     *      - 若该逐笔时间 > last_tick_ms[code]，说明在“上一次 quote 之后”，
     *        将其暂存到 interval 的 pending 容器；
     *      - 等下一条 quote 来时，在 on_quote 中统一发布 interval + tick 级指标。
     *  3）无论如何，都会喂给 NmsBucketAggregator，用于时间桶聚合。
     */
    void TickTransOrders::on_tick(const CombinedTick& x) {
        if (x.kind == CombinedKind::Trade) {
            // 处理成交逻辑
            Transaction t{};
            t.instrument_id = x.instrument_id;
            t.data_time_ms  = x.data_time_ms;
            t.main_seq      = x.main_seq;
            t.price         = x.price;
            t.side          = x.side;
            t.volume        = x.volume;
            t.bid_no        = x.bid_no;
            t.ask_no        = x.ask_no;

            ensure_code(t.instrument_id);

            // 处理 tick 间隔内的成交信息
            if (_cfg.emit_tick_interval) {
                auto lt = _last_tick_ms[t.instrument_id];
                if (lt > 0 && t.data_time_ms > lt) {
                    _interval_trans_pending[t.instrument_id].push_back(t);
                }
            }

            // 将成交数据加入桶聚合器
            _agg[t.instrument_id].on_transaction(t);
            maybe_flush_and_publish(t.instrument_id, t.data_time_ms);

        } else if (x.kind == CombinedKind::Order) {
            // 处理委托逻辑（新增和撤单都归为 Order 统一处理）
            Entrust e{};
            e.instrument_id = x.instrument_id;
            e.data_time_ms  = x.data_time_ms;
            e.main_seq      = x.main_seq;
            e.price         = x.price;
            e.side          = x.side;
            e.volume        = x.volume;
            e.order_id      = x.order_id;

            ensure_code(e.instrument_id);

            // 处理 tick 间隔内的委托信息
            if (_cfg.emit_tick_interval) {
                auto lt = _last_tick_ms[e.instrument_id];
                if (lt > 0 && e.data_time_ms > lt) {
                    _interval_orders_pending[e.instrument_id].push_back(e);
                }
            }

            // 将委托数据加入桶聚合器
            _agg[e.instrument_id].on_entrust(e);
            maybe_flush_and_publish(e.instrument_id, e.data_time_ms);
        } else {
            // 如果是 Empty 类型，表示不处理
        }
    }


    /**
     * @brief 主动强制 flush 指定代码当前未输出的时间桶
     *
     * @param code 标的代码
     * @return true  存在未输出桶且已成功发布
     * @return false 没有未输出桶或代码不存在
     *
     * 场景：
     *  - 一般在“日切 / 收盘 / 回测结束”时调用，确保最后一个桶也被输出。
     */
    bool TickTransOrders::force_flush(const std::string& code){
        auto it = _agg.find(code);
        if (it == _agg.end()) return false;

        BucketOutputs out;
        if (it->second.force_flush(out)){
            publish_bucket(code, out);
            return true;
        }
        return false;
    }

} // namespace factorlib
