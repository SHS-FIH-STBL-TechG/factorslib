// include/utils/nms_bucket_aggregator.h
#pragma once
#include "utils/types.h"
#include "utils/trading_time.h"

namespace factorlib {

    /**
     * @brief 事件→N 毫秒时间桶的聚合器（无滑窗；跨桶即产出）
     */
    class NmsBucketAggregator {
    public:
        explicit NmsBucketAggregator(int64_t bucket_ms=1000);

        /// 设置时间桶宽度（毫秒）
        void set_bucket_ms(int64_t ms);

        /// 喂入一条行情（用于增量计算 amount/volume 和更新 midprice）
        void on_quote(const QuoteDepth& q);

        /// 喂入一条成交（仅用于桶切片，不影响 amount/volume）
        void on_transaction(const Transaction& t);

        /// 喂入一条委托（仅用于桶切片）
        void on_entrust(const Entrust& e);

        /**
         * @brief 若 now_ms 已跨越当前桶的尾，则产出上一桶到 out，并开启新桶
         * @return 是否产出了一个桶
         */
        bool flush_if_crossed(int64_t now_ms, BucketOutputs& out);

        /**
         * @brief 强制产出当前桶
         * @return 是否有桶被产出
         */
        bool force_flush(BucketOutputs& out);

        /**
         * @brief 确保当前时间桶已建立；若跨越，则产出旧桶
         */
        bool ensure_bucket(int64_t ts_ms, BucketOutputs& out);

    private:
        int64_t _bucket_ms{1000};
        bool _has_bucket{false};
        BucketOutputs _cur{};

        // 用于增量计算的上一次行情记忆
        bool _has_last_quote{false};
        int _last_trading_day{0};
        uint64_t _last_volume{0};
        double _last_turnover{0.0};

        void start_new_bucket(int64_t new_start);
    };

} // namespace factorlib