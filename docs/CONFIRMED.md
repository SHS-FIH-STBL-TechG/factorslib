
# 因子库确认记录（CONFIRMED）

## 最新确认（本次对话）
- **日志宏位置**：`include/utils/log.h`（实现于 `src/utils/log.cpp`）。
- **DataBus 扩展**：采用**模板化 Channel**，并支持“环形历史 + 时间戳与股票编号匹配”。
  - D-1：环形历史（按数量淘汰），键为 `(topic, code)`，每条数据含精确 `ts_ms`；`get_by_time_exact(topic, code, ts_ms)` 精确匹配读取，避免拿错。
  - D-2：提供 **Pull + 订阅** 两种访问方式：
    - Pull：`get_latest / get_last_n / get_by_time_exact`；
    - 订阅：`subscribe(topic, code, callback)`，发布时触发；
    - `topic`：层级命名，如 `zyd/amount`；`code`：股票/合约代码，如 `000001.SZ`。
  - D-3：**模板化**（类型安全）：每个 `topic` 绑定一个类型 `T`，`register_topic<T>()` 后只能以 `T` 发布/读取；
    - 示例见下（同时给“统一结构”对比示例）。
  - D-4：仅按**数量**淘汰（环形容量 N 可配）。
  - D-5：**层级命名**：如 `zyd/amount`, `zyd/tick/trans`。
  - D-6：并发模型 **A**：全局互斥保护，简单稳定。
- **目录结构**：
  - `src/basic_factors/`：基础因子（**会写入 DataBus**，供他因子复用）；
  - `src/advanced_factors/`：复杂因子（可能读取基础因子，但**不写入 DataBus**）；
  - `include/utils/`：公共计算类（`utils.h`）与数据传递类（`databus.h`）、日志（`log.h`）；
  - `src/utils/`：工具类实现（`log.cpp` 等）。

## 既有确认（沿用）
- 时间单位：**毫秒**。
- 聚合粒度：**N 毫秒桶**（默认 1000ms）。
- 事件驱动：**跨桶就产出上一桶**；无窗口、不使用 `raw_event_capacity` / `bucket_window_size`。
- 0109 因子“翻译”产出：
  - `zyd/amount`（double，桶内成交额增量之和）
  - `zyd/volume`（int64_t，桶内成交量增量之和）
  - `zyd/midprice`（double，桶内最后一笔 tick 的中价）
  - `zyd/tick/trans`（std::vector<Transaction>）
  - `zyd/tick/orders`（std::vector<Entrust>）
- 测试：使用 googletest/gmock；数值断言；测试数据在 `tests/utils/`。


### 新增：因子间等待机制（A 等 B，最多 1s）
- DataBus 提供阻塞式读取：
  - `wait_for_time_exact<T>(topic, code, ts_ms, out, timeout_ms=1000)`：等待到**精确时间戳**的数据；超时返回 false（A 本次跳过）。
  - `wait_for_time_at_least<T>(topic, code, ts_ms, out, timeout_ms=1000)`：等待**时间戳不早于 ts_ms** 的最新数据。
- 发布时机：基础因子在产出桶时以**桶结束时间**为 ts 进行 `publish`；A 因子依赖 B 因子时可以用同一 ts 对齐。
