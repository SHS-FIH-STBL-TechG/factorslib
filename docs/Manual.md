
# factorlib 框架说明书（上手即用版）

> 目标：让从未接触过本框架的人**10~30 分钟完成**“拉起测试 → 写一个基础因子 → 被复杂因子复用”的最小闭环。

## 目录

- [1. 框架定位与术语](#1-框架定位与术语)  
- [2. 目录结构与文件说明](#2-目录结构与文件说明)  
- [3. 快速开始（构建与运行测试）](#3-快速开始构建与运行测试)  
- [4. 核心概念](#4-核心概念)  
  - [4.1 时间与交易时段](#41-时间与交易时段)  
  - [4.2 N 毫秒桶聚合](#42-n-毫秒桶聚合)  
  - [4.3 基础因子与复杂因子](#43-基础因子与复杂因子)  
  - [4.4 模板化 DataBus（数据传递总线）](#44-模板化-databus数据传递总线)  
  - [4.5 因子间等待（A 等 B 最多 1s）](#45-因子间等待a-等-b-最多-1s)  
  - [4.6 日志](#46-日志)  
- [5. 公共类型与工具](#5-公共类型与工具)  
- [6. 基础因子：从 0 到 1](#6-基础因子从-0-到-1)  
  - [6.1 注册 topic](#61-注册-topic)  
  - [6.2 编写因子类](#62-编写因子类)  
  - [6.3 发布产出（publish）](#63-发布产出publish)  
- [7. 复杂因子：复用基础因子结果](#7-复杂因子复用基础因子结果)  
- [8. 单元测试（googletest/gmock）](#8-单元测试googletestgmock)  
- [9. 约定与最佳实践](#9-约定与最佳实践)  
- [10. 常见问题（FAQ）](#10-常见问题faq)  
- [11. 变更记录与设计确认](#11-变更记录与设计确认)

---

## 1. 框架定位与术语

- **factorlib**：一个极简、可测试的**因子开发框架**。目标是**解耦**业务因子计算与外围系统（消息总线、历史文件、生产权限），让因子**本地即可开发/联调/单测**。
- **基础因子（basic_factors）**：直接读取行情/交易/委托数据，**计算并发布**结果到 DataBus，供其它因子复用。
- **复杂因子（advanced_factors）**：**只读取**基础因子的结果，可以做进一步计算，但**不向 DataBus 写入**（避免环依赖、爆炸图）。
- **DataBus**：类型安全的数据传递总线（模板化），支持**层级 topic**、**按 code（股票/合约）**与**时间戳**存取，提供**环形历史**与**阻塞等待（1s 超时）**。
- **N 毫秒桶聚合**：把事件（tick/交易/委托）按 N 毫秒对齐到“桶”（默认 1000ms=1s），**跨桶即产出**上一桶结果（无滑窗）。  

> 本框架已“翻译”了一个示例基础因子 `tick_trans_orders`（来自原 0109），产出：  
> - `zyd/amount`：每桶成交额增量之和  
> - `zyd/volume`：每桶成交量增量之和  
> - `zyd/midprice`：每桶最后一笔 tick 的中价  
> - `zyd/tick/trans`：该桶内交易列表  
> - `zyd/tick/orders`：该桶内委托列表

---

## 2. 目录结构与文件说明

```
include/utils/
  log.h          # 日志宏 TSET_INFO/TSET_WARN/TSET_ERROR
  utils.h        # 公共数据类型、NmsBucketAggregator（事件→N 毫秒桶）
  databus.h      # 模板化 DataBus（register_topic/publish/get/subscribe/wait）

src/
  basic_factors/
    tick_trans_orders.h/.cpp    # 示例基础因子（已翻译自 0109）
  advanced_factors/
    (可放你的复杂因子)
  utils/
    log.cpp
    utils.cpp

tests/
  test_bus_and_factor.cpp       # gtest：验证 DataBus 与 0109 的产出数值
  test_wait.cpp                 # gtest：验证等待机制（发布晚到也能拿到）
  utils/
    data_gen.h                  # 测试数据生成（构造可预期的行情序列）

third_party/googletest/         # 你放 gtest/gmock（源码或静态库）
docs/
  CONFIRMED.md                  # 我们已确认的全部口径
  Manual.md                     # 本文档（上手手册）
README.md                       # 快速构建说明
```

---

## 3. 快速开始（构建与运行测试）

> 前置：请先把 googletest 放入 `third_party/googletest`（可放源码或 `libgtest.a` 静态库）。

### 方式 A：链接静态库（推荐）

```bash
g++ -std=c++17 -O2 -Iinclude -Isrc -Ithird_party/googletest/include \
  src/utils/log.cpp src/utils/utils.cpp \
  src/basic_factors/tick_trans_orders.cpp \
  tests/test_bus_and_factor.cpp tests/test_wait.cpp \
  -Lthird_party/googletest/lib -lgtest -lgtest_main -lpthread -o tests/run_tests

./tests/run_tests
```

### 方式 B：直接编译 googletest 源码

```bash
GTEST_SRC=third_party/googletest
g++ -std=c++17 -O2 -Iinclude -Isrc -I$GTEST_SRC/googletest/include \
  src/utils/log.cpp src/utils/utils.cpp \
  src/basic_factors/tick_trans_orders.cpp \
  tests/test_bus_and_factor.cpp tests/test_wait.cpp \
  $GTEST_SRC/googletest/src/gtest-all.cc \
  -lpthread -o tests/run_tests

./tests/run_tests
```

---

## 4. 核心概念

### 4.1 时间与交易时段
- **时间单位**：毫秒（`int64_t`，UNIX epoch ms）。
- **交易时段**（默认 A 股）：`[09:30, 11:30)` 和 `[13:00, 15:00)`；仅在时段内入账。  
  - 函数：`bool in_trading_session_ms(int64_t ms)`（`include/utils/utils.h`）

### 4.2 N 毫秒桶聚合
- **为什么**：把零散事件按固定时长（默认 1000ms）聚到“时间桶”，便于统一产出与下游消费。
- **触发**：当新事件时间戳**跨过当前桶的尾**，就**先产出旧桶**，再开启新桶。
- **默认桶尾作为产出时间戳**（`bucket_end_ms`），下游按这个 ts 精确获取。

### 4.3 基础因子与复杂因子
- **基础因子（basic_factors）**  
  - 直接接收 quote/transaction/entrust。  
  - **写入 DataBus**：将其产出的指标作为可复用的“公共数据池”。  
- **复杂因子（advanced_factors）**  
  - **只读** DataBus（可阻塞等待 1s）。  
  - **不写入** DataBus，避免环依赖、避免复杂因子间相互复用。

### 4.4 模板化 DataBus（数据传递总线）
- **层级 topic** + **code** + **时间戳（ms）**  
- 每个 topic 绑定唯一类型 T：`register_topic<T>(topic, capacity)`  
- 发布：`publish<T>(topic, code, ts_ms, value)`  
- 读取：  
  - `get_latest<T>(...)`：拿最近一条  
  - `get_last_n<T>(..., n)`：拿最近 n 条  
  - `get_by_time_exact<T>(..., ts_ms)`：按**精确 ts**取（**避免拿错**）
- **环形历史**：仅按**数量**淘汰，避免无限增长。

### 4.5 因子间等待（A 等 B 最多 1s）
- **场景**：A 因子依赖 B 的产出；如果 B 还没算完，A **等待最多 1s**，仍没有则**本次跳过**。
- API：  
  - `wait_for_time_exact<T>(topic, code, ts_ms, out, timeout_ms=1000)`  
  - `wait_for_time_at_least<T>(topic, code, ts_ms, out, timeout_ms=1000)`  
- 实现细节：内部用 `condition_variable` 通知；**线程安全**，不会忙等占满 CPU。

### 4.6 日志
- 头文件：`include/utils/log.h`  
- 宏：`TSET_INFO("x=%d", x)` → 输出格式：`[INFO] [Class::Func] message`  
- 在 Linux/Clang/GCC 下使用 `__PRETTY_FUNCTION__`，自动带类名与函数名。

---

## 5. 公共类型与工具

`include/utils/utils.h` 提供以下**数据结构**和**聚合器**：

```cpp
struct QuoteDepth {
  std::string instrument_id;  // 代码，如 "000001.SZ"
  int64_t data_time_ms;       // 时间戳（ms）
  int trading_day;            // 交易日（可选，用于跨日增量）
  uint64_t volume;            // 成交量（累计）
  double turnover;            // 成交额（累计）
  double bid_price, ask_price;// 买一/卖一
};

struct Transaction { /* price, volume, main_seq, ... */ };
struct Entrust     { /* price, volume, main_seq, ... */ };

struct BucketOutputs {
  double amount_sum;          // 桶内成交额增量之和
  int64_t volume_sum;         // 桶内成交量增量之和
  double midprice_last;       // 桶内最后一笔中价
  std::vector<Transaction> trans;  // 桶内交易
  std::vector<Entrust> orders;     // 桶内委托
  int64_t bucket_start_ms, bucket_end_ms; // 桶范围（尾是开区间）
};
```

**NmsBucketAggregator**（事件→Nms 桶）：
- `set_bucket_ms(N)`：设定桶时长（默认 1000ms）  
- `on_quote / on_transaction / on_entrust(...)`：推进聚合  
- `flush_if_crossed(now_ms, out)`：若跨桶则产出上一桶  
- `force_flush(out)`：强制产出当前桶（例如收盘或测试）

---

## 6. 基础因子：从 0 到 1

### 6.1 注册 topic

> 基础因子会**写入** DataBus，因此先注册 topic（指定类型和历史容量）。

```cpp
#include "utils/databus.h"

using namespace factorlib;

void RegisterMyTopics() {
  auto& bus = DataBus::instance();
  bus.register_topic<double>("demo/alpha", 120);         // 最近 120 条
  bus.register_topic<int64_t>("demo/beta", 120);
}
```

### 6.2 编写因子类

> 建议参考已有 `src/basic_factors/tick_trans_orders.*`；最小骨架如下：

```cpp
// src/basic_factors/my_factor.h
#pragma once
#include <string>
#include <unordered_map>
#include "utils/utils.h"
#include "utils/databus.h"
#include "utils/log.h"

namespace factorlib {

struct MyFactorConfig { int64_t bucket_size_ms = 1000; };

class MyFactor {
public:
  explicit MyFactor(const MyFactorConfig& cfg, std::vector<std::string> codes)
   : _cfg(cfg), _codes(std::move(codes)) {}

  static void register_topics(size_t cap=120) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>("demo/alpha", cap);
  }

  void on_quote(const QuoteDepth& q) {
    if (q.instrument_id.empty() || !in_trading_session_ms(q.data_time_ms)) return;
    ensure_code(q.instrument_id);
    maybe_flush_and_publish(q.instrument_id, q.data_time_ms);
    _agg[q.instrument_id].on_quote(q);
  }
  // 如果需要交易/委托，仿照 on_transaction/on_entrust 即可

  bool force_flush(const std::string& code) {
    auto it = _agg.find(code);
    if (it==_agg.end()) return false;
    BucketOutputs out;
    if (it->second.force_flush(out)) { publish_bucket(code, out); return true; }
    return false;
  }

private:
  MyFactorConfig _cfg;
  std::vector<std::string> _codes;
  std::unordered_map<std::string, NmsBucketAggregator> _agg;

  void ensure_code(const std::string& code) {
    if (_agg.find(code)==_agg.end()) _agg.emplace(code, NmsBucketAggregator(_cfg.bucket_size_ms));
  }
  void maybe_flush_and_publish(const std::string& code, int64_t now_ms) {
    BucketOutputs out;
    auto& ag = _agg.at(code);
    if (ag.flush_if_crossed(now_ms, out)) publish_bucket(code, out);
  }
  void publish_bucket(const std::string& code, const BucketOutputs& out) {
    auto ts = out.bucket_end_ms; // 以桶尾时间为产出时间戳
    DataBus::instance().publish<double>("demo/alpha", code, ts, out.midprice_last /* 举例 */);
    TSET_INFO("publish %s [%lld,%lld) val=%.6f", code.c_str(),
              (long long)out.bucket_start_ms, (long long)out.bucket_end_ms, out.midprice_last);
  }
};

} // namespace factorlib
```

### 6.3 发布产出（publish）

- **何时**：每次跨桶时（或 `force_flush`）发布一次。  
- **发布哪些**：按你的业务定义。示例中把 `out.midprice_last` 写入 `demo/alpha`。  
- **时间戳**：统一采用**桶尾时间**，便于下游对齐等待。

---

## 7. 复杂因子：复用基础因子结果

> 复杂因子**不写入 DataBus**，仅**读取**基础因子的结果。  
> 约定：如果依赖的基础因子**还没产出**，复杂因子**等待最多 1s**，超时就**跳过当前计算**。

```cpp
// src/advanced_factors/my_advanced.h
#pragma once
#include "utils/databus.h"
#include "utils/log.h"

namespace factorlib {

class MyAdvanced {
public:
  // 从基础因子读数据并计算
  bool calc(const std::string& code, int64_t ts_ms, double& out_val) {
    // 等待基础因子的产出（精确到该 ts）
    double mid = 0.0;
    bool ok = DataBus::instance().wait_for_time_exact<double>(
      "zyd/midprice", code, ts_ms, mid, /*timeout_ms*/ 1000
    );
    if (!ok) {
      TSET_WARN("skip calc: midprice not ready code=%s ts=%lld",
                code.c_str(), (long long)ts_ms);
      return false;
    }
    // 读取另一条（可选）："zyd/amount"
    double amt = 0.0;
    ok = DataBus::instance().wait_for_time_exact<double>(
      "zyd/amount", code, ts_ms, amt, 1000
    );
    if (!ok) return false;

    // 你的计算：
    out_val = mid > 0 ? amt / mid : 0.0;

    TSET_INFO("calc ok code=%s ts=%lld out=%.6f", code.c_str(), (long long)ts_ms, out_val);
    return true;
  }
};

} // namespace factorlib
```

> 注意：**只有基础因子**写入 DataBus，**复杂因子不写**，以免出现复杂因子间相互依赖。

---

## 8. 单元测试（googletest/gmock）

- test 目录已提供两个示例：  
  - `test_bus_and_factor.cpp`：验证 DataBus 模板化 + 0109 的产出数值（amount/volume/midprice 与手工预期一致）。  
  - `test_wait.cpp`：演示发布晚到 100ms 的情况下，`wait_for_time_exact` 在 1s 内能拿到数据。  
- 构建方式见[快速开始](#3-快速开始构建与运行测试)。

---

## 9. 约定与最佳实践

1. **topic 命名**用**层级**：`zyd/amount`、`zyd/tick/trans`，便于分组与检索。  
2. **类型安全**：注册 `register_topic<T>()` 后，`publish/get/wait` 都**必须**使用相同 `T`。  
3. **时间戳统一**：产出统一用**桶尾时间**，下游用同一个 ts 对齐（精确取/等待）。  
4. **环形历史仅按数量淘汰**：避免无限存储；默认 120，可按需调整。  
5. **基础因子进池子，复杂因子不进池子**：避免因子环依赖；复杂因子之间**不要互相读写**。  
6. **等待 1s 超时**：复杂因子等待基础因子不足 1s；超时**跳过**，不要阻塞主流程。  
7. **日志**：默认 INFO/WARN/ERROR，**不要输出过量**；大循环里谨慎打印。  
8. **线程安全**：DataBus 内部加锁与条件变量；你自己的因子若涉及多线程，外层仍需规约访问。  
9. **跨日处理**：以 `trading_day` 判断增量归零；若拿不到 `trading_day`，可在 `utils.cpp` 中根据自然日回绕处理（需要时再扩展）。

---

## 10. 常见问题（FAQ）

**Q1：为什么用模板化 DataBus，而不是统一结构？**  
A：模板化能**编译期保证类型正确**，避免“拿错字段/读错类型”的隐性错误；这也是明确要求。

**Q2：topic 与 code 是什么？**  
A：`topic` 是**数据主题**（如 `zyd/amount`），`code` 是**股票/合约**（如 `000001.SZ`）。两者共同定位一条数据序列。

**Q3：等待 1s 是阻塞的吗？会卡死吗？**  
A：阻塞等待最多 1s，内部使用 `condition_variable`，**有数据到达立即唤醒**，不会忙等；超时返回 `false`，由调用方**跳过**。

**Q4：桶内没有 quote 只有交易/委托会不会漏产出？**  
A：`on_transaction/on_entrust` 也会触发跨桶检查，一般不会漏；如需“定时心跳式”产出，可加一个**外部心跳**调用 `flush_if_crossed(now)` 的层。  

**Q5：能改成窗口（W 个桶）才产出吗？**  
A：当前不需要；未来如新增窗口型因子，可在 `utils` 新增 `SlidingWindowBucket`，并遵循“窗口满才产出”的语义。

---

## 11. 变更记录与设计确认

- **统一时间单位为 ms**；**N 毫秒桶**（默认 1000ms）；**跨桶产出**。  
- **基础因子 → DataBus**，**复杂因子只读**。  
- **模板化 DataBus**：`register_topic<T> / publish / get_by_time_exact / wait_for_time_exact`。  
- **环形历史按数量淘汰**；**层级命名**。  
- **因子间等待**：A 等 B 最多 1s，超时跳过。  
- **日志宏**在 `include/utils/log.h`；输出 `[级别] [类名::函数名] 信息`。  
- `docs/CONFIRMED.md` 持续同步**每次对齐的口径**，作为开发准绳。

---

# 附录：常用代码片段速查

**注册 & 发布**
```cpp
auto& bus = DataBus::instance();
bus.register_topic<double>("zyd/amount", 120);
bus.publish<double>("zyd/amount", "000001.SZ", /*ts_ms*/ 1730964601000, 40000.0);
```

**读取（最新/最近 n/精确 ts）**
```cpp
double a=0; int64_t ts=0;
bus.get_latest<double>("zyd/amount", "000001.SZ", a, &ts);
auto vec = bus.get_last_n<double>("zyd/amount", "000001.SZ", 5);

double exact=0;
bool ok = bus.get_by_time_exact<double>("zyd/amount","000001.SZ", 1730964601000, exact);
```

**等待（最多 1s）**
```cpp
double mid=0;
bool ok = bus.wait_for_time_exact<double>("zyd/midprice","000001.SZ", 1730964601000, mid, 1000);
if (!ok) /* 跳过 */;
```

**基础因子：跨桶产出**
```cpp
BucketOutputs out;
if (aggregator.flush_if_crossed(now_ms, out)) {
  // 用 out.bucket_end_ms 作为 ts 发布
}
```

**日志**
```cpp
TSET_INFO("calc done code=%s ts=%lld v=%.6f", code.c_str(), (long long)ts, v);
```


---
## 使用 CMake 构建（新增）

> 本仓库已内置 CMake：根目录有 `CMakeLists.txt`，`third_party/googletest/` 下有最小 `CMakeLists.txt`。
> 你只需把官方 googletest 的 `googletest/include` 和 `googletest/src` 放入对应目录即可。

### 一次性构建与运行测试
```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j
ctest --output-on-failure
```
生成的可执行文件在 `build/` 目录（`run_tests`）。
