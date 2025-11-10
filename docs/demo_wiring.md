# Demo 到因子的数据接入（Route B：库内桥接）

## 使用方式（不改动 demo 原有逻辑，只在回调内各加一行）
> 下面说明**demo 应该如何把数据传到因子**、以及**demo 使用因子时应添加的代码**。

1. 在 `AppDemo.cpp` 顶部引入：
```cpp
#include "factorlib/bridge/ingress.h"
#include "ifactor.h"
```
2. 在初始化处注册你的因子（示例两条）：
```cpp
std::vector<std::shared_ptr<factorlib::IFactor>> fs;
fs.emplace_back(std::make_shared<TickTransOrders>>());
fs.emplace_back(std::make_shared<GaussianCopulaFactor>>());
factorlib::bridge::set_factors(fs);
```
3. 在三个回调中各加一行：
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

## 原理
- demo 的原始结构体（`SnapshotStockSH / OrdAndExeInfo / BasicandEnhanceKLine`）将由库内 `DataAdapter` 转换为统一类型：
  - `SnapshotStockSH → QuoteDepth`
  - `OrdAndExeInfo → Transaction/Entrust`（依据 `TradeQty>0` 判定为成交/委托）
  - `BasicandEnhanceKLine → Bar`
- `ingress` 在库内完成“转化 → 分发到所有已注册因子”的动作；因子回调接口为：
  - `on_quote(const QuoteDepth&)`
  - `on_transaction(const Transaction&)`
  - `on_entrust(const Entrust&)`
  - `on_bar(const Bar&)`

## 备注
- 本路线不改变 `IFactor` 的签名；demo 不需要了解库内的统一类型字段。
- 将来替换行情源时，仅需在 `DataAdapter`/`ingress` 层调整映射，不影响 demo 和因子。
