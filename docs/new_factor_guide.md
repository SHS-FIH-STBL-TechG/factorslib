# 新增因子开发与接入指南（开箱即用）

> 目标：在现有 `factors_lib` 中新增一个因子（示例名：`MyAlphaMomentum`），并让 demo 把数据传到该因子参与计算与发布结果。  
> 适用数据回调：`on_quote`（快照）、`on_transaction`（逐笔成交）、`on_entrust`（逐笔委托）、`on_bar`（K 线）。  
> 路线：**不入侵 demo 业务逻辑**，只在初始化处注册因子；数据接线仍走 `ingest_*`（bridge 内完成转换+分发）。

---

## 0. 目录与关键约定

- 因子接口：`factors_lib/include/factorlib/core/ifactor.h`（定义 `on_quote/on_transaction/on_entrust/on_bar/force_flush` 等）  
- 公共类型：`factors_lib/include/factorlib/core/types.h`（`QuoteDepth/Transaction/Entrust/Bar` 等）  
- 桥接入口：`factors_lib/include/factorlib/bridge/ingress.h`（`ingest_snapshot/ingest_ont/ingest_kline/set_factors`）  
- 数据适配：`factors_lib/include/factorlib/utils/data_adapter.h`、`factors_lib/src/utils/data_adapter.cpp`（demo 类型 → 统一类型）  
- 推荐放置新因子：  
  - 头文件：`factors_lib/include/factorlib/factors/basic/my_alpha_momentum.h`  
  - 源文件：`factors_lib/src/factors/basic/my_alpha_momentum.cpp`  
- 发布主题（DataBus）示例：`alpha/momentum/*`（遵循你现有命名规范即可）

---

## 1. 新增因子代码

### 1.1 头文件：`factors_lib/include/factorlib/factors/basic/my_alpha_momentum.h`

```cpp
#pragma once
#include "factorlib/core/ifactor.h"
#include "factorlib/core/types.h"
#include <deque>
#include <string>

/**
 * 因子：MyAlphaMomentum
 * 思路：维护一个固定窗口的 K 线收盘价，计算动量（最近收盘价 / 窗口均值 - 1），
 *       并通过 DataBus 发布结果。
 */
class MyAlphaMomentum : public factorlib::BaseFactor {
public:
    explicit MyAlphaMomentum(std::string instrument_filter = "")
        : instrument_filter_(std::move(instrument_filter)) {}

    // 可按需覆盖以下任意回调；本示例主用 on_bar（K线）
    void on_bar(const Bar& b) override;              // ← K线驱动
    void on_quote(const QuoteDepth& q) override;     // 可选：若需要用快照计算
    void on_transaction(const Transaction& t) override {}  // 暂不用
    void on_entrust(const Entrust& e) override {}          // 暂不用

    // 收尾：需要时强制输出
    bool force_flush() override;

private:
    void push_close(double close, int64_t ts_ms);
    double compute_momentum() const;

private:
    // 配置
    static constexpr size_t kWindow = 20; // 20根K线窗口

    // 状态
    std::deque<double> closes_;
    int64_t last_ts_ms_ = 0;

    // 过滤：只接指定代码（留空则不过滤）
    std::string instrument_filter_;
};
```

### 1.2 源文件：`factors_lib/src/factors/basic/my_alpha_momentum.cpp`

```cpp
#include "factorlib/factors/basic/my_alpha_momentum.h"
#include "factorlib/core/types.h"
#include "factorlib/core/databus.h"       // DataBus 发布工具等
#include <numeric>
#include <cmath>

// 主题命名：按你的规范调整
static const char* kTopicMomentum = "alpha/momentum/value";
static const char* kTopicTS       = "alpha/momentum/ts";

void MyAlphaMomentum::push_close(double close, int64_t ts_ms) {
    closes_.push_back(close);
    if (closes_.size() > kWindow) closes_.pop_back();
    last_ts_ms_ = ts_ms;
}

double MyAlphaMomentum::compute_momentum() const {
    if (closes_.empty()) return 0.0;
    double sum = std::accumulate(closes_.begin(), closes_.end(), 0.0);
    double mean = sum / static_cast<double>(closes_.size());
    double last = closes_.back();
    if (mean == 0.0) return 0.0;
    return last / mean - 1.0;
}

void MyAlphaMomentum::on_bar(const Bar& b) {
    if (!instrument_filter_.empty() && b.instrument_id != instrument_filter_) return;
    push_close(b.close, b.data_time_ms);
    if (closes_.size() < 2) return;

    const double mom = compute_momentum();
    factorlib::DataBus::publish<double>(kTopicMomentum, mom);
    factorlib::DataBus::publish<int64_t>(kTopicTS, last_ts_ms_);
}

void MyAlphaMomentum::on_quote(const QuoteDepth& q) {
    (void)q; // 示例不使用。如需，用中间价 (bid+ask)/2 推进窗口。
}

bool MyAlphaMomentum::force_flush() {
    if (closes_.empty()) return false;
    const double mom = compute_momentum();
    factorlib::DataBus::publish<double>(kTopicMomentum, mom);
    factorlib::DataBus::publish<int64_t>(kTopicTS, last_ts_ms_);
    return true;
}
```

---

## 2. CMake 增量（库内）

打开 `factors_lib/CMakeLists.txt`，将新源文件加入对应目标（按你工程实际目标名称；若使用 GLOB 收集则可省略）：

```cmake
# 示例：把新因子源加入 factor_basic 目标
target_sources(factor_basic PRIVATE
    src/factors/basic/my_alpha_momentum.cpp
)
```

---

## 3. 单元测试（建议）

`factors_lib/tests/my_alpha_momentum_test.cpp`：

```cpp
#include <gtest/gtest.h>
#include "factorlib/factors/basic/my_alpha_momentum.h"
#include "factorlib/core/types.h"

TEST(MyAlphaMomentumTest, BasicMomentum) {
    MyAlphaMomentum f(/*instrument_filter=*/"600000.SH");

    auto mk = [](double close, int64_t t) {
        Bar b; b.instrument_id = "600000.SH"; b.close = close; b.data_time_ms = t;
        return b;
    };

    f.on_bar(mk(10.0, 1000));
    f.on_bar(mk(10.1, 2000));
    f.on_bar(mk(10.2, 3000));

    // 断言：你可用现有 DataBus 工具读取发布值做校验
    SUCCEED();
}
```

> 如果你的测试套件里有 DataBus 的等待/收集工具（如 `WaitTopic/get_last`），可以直接读取 `alpha/momentum/value` 做数值断言。

---

## 4. demo 侧需要增加什么？

> 路线 B：**不改动 demo 业务逻辑**。只需两处：**初始化注册** + **（若未接线）回调一行接线**。

### 4.1 初始化处注册新因子

```cpp
#include "factorlib/bridge/ingress.h"
#include "factorlib/core/ifactor.h"
#include "factorlib/factors/basic/my_alpha_momentum.h"  // ← 新增

// 初始化时：
std::vector<std::shared_ptr<factorlib::IFactor>> fs;
fs.emplace_back(std::make_shared<TickTransOrders>());
fs.emplace_back(std::make_shared<GaussianCopulaFactor>());
fs.emplace_back(std::make_shared<MyAlphaMomentum>("600000.SH")); // ← 新增注册
factorlib::bridge::set_factors(fs);
```

### 4.2 若未接线：回调各加一行（已接线则无需重复）

```cpp
void AppDemo::RecvSnapStruct(const std::vector<SnapshotStockSH>& v) {
    factorlib::bridge::ingest_snapshot(v);
}
void AppDemo::RecvOntStruct(const std::vector<OrdAndExeInfo>& v) {
    factorlib::bridge::ingest_ont(v);
}
void AppDemo::RecvKLineData(const std::vector<BasicandEnhanceKLine>& v) {
    factorlib::bridge::ingest_kline(v);
}
```

---

## 5. 文档同步

- `factors_lib/docs/demo_wiring.md`：新增“小节：接入 MyAlphaMomentum”，贴上**注册代码**与**发布主题**。  
- `README.md` / `CODEMAP.md`：把新文件加入目录与说明，并在 CODEMAP 为新增函数加**功能/参数/返回**说明。

---

## 6. 最小上线清单（Checklist）

- [ ] 新增文件：  
  - [ ] `include/factorlib/factors/basic/my_alpha_momentum.h`  
  - [ ] `src/factors/basic/my_alpha_momentum.cpp`  
  - [ ] `tests/my_alpha_momentum_test.cpp`（建议）  
- [ ] CMake：`my_alpha_momentum.cpp` 已加入目标（或目录自动收集）。  
- [ ] demo：初始化已 `emplace_back(std::make_shared<MyAlphaMomentum>(...))`。  
- [ ] demo 回调：`ingest_snapshot / ingest_ont / ingest_kline` 已存在。  
- [ ] 文档：`demo_wiring.md`/`README.md`/`CODEMAP.md` 已补新因子说明。  
- [ ] 运行 `run_tests`：全部通过。

---

## 7. 扩展建议（可选）

- **盘口驱动**：在 `on_quote` 用中间价 `(bid+ask)/2` 推进窗口。  
- **逐笔驱动**：在 `on_transaction` 维护区间 VWAP 或交易强度指标。  
- **多代码并行**：维护 `unordered_map<std::string, deque<double>>`，每个 `instrument_id` 一套窗口。  
- **主题设计**：按代码维度发布，形如 `alpha/momentum/{instrument_id}`。
