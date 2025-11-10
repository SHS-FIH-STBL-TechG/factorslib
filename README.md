# FactorLib - 量化因子计算框架

## 📖 项目概述

FactorLib 是一个**专业级量化因子计算框架**，专为高频量化交易场景设计。它提供了一个标准化的因子开发、管理和数据流处理平台，让量化研究员能够专注于因子逻辑本身，而不必担心数据流、时间对齐和依赖管理等复杂工程问题。

### 🎯 核心设计理念

1. **统一接口规范**：所有因子都实现统一的 `IFactor` 接口，确保一致的开发体验
2. **数据总线机制**：通过 `DataBus` 实现因子间的数据共享和依赖管理
3. **时间桶聚合**：自动处理高频数据的时序对齐问题，确保计算一致性
4. **最小入侵设计**：通过桥接层与外部系统解耦，易于集成到现有系统
5. **类型安全**：模板化的数据总线确保数据类型安全
6. **模块化架构**：清晰的职责分离，便于维护和扩展

## 🏗️ 项目架构详解

### 优化后的目录结构

```
factors_lib/
├── include/                    # 公共头文件（对外接口）
│   ├── factor_manager.h       # 因子管理器 - 统一管理所有因子实例
│   ├── ifactor.h              # 因子接口定义 - 所有因子的基类
│   ├── factorlib/bridge/
│   │   └── ingress.h          # 数据入口桥接层 - 最小化外部依赖
│   └── utils/                 # 工具库（模块化设计）
│       ├── types.h            # 公共数据类型定义 - 标准数据结构
│       ├── trading_time.h     # 交易时间工具 - 时段判断和日期处理
│       ├── math_utils.h       # 数学计算工具 - 统计和数学函数
│       ├── nms_bucket_aggregator.h # 时间桶聚合器 - 时序对齐引擎
│       ├── data_adapter.h     # 数据格式适配器 - 外部数据→内部标准格式
│       ├── databus.h          # 数据总线（核心） - 因子间通信枢纽
│       └── log.h              # 日志系统 - 统一日志接口
├── src/                       # 实现文件
│   ├── basic_factors/         # 基础因子实现
│   │   ├── tick_trans_orders.cpp  # 基础因子示例：时间桶聚合
│   │   └── tick_trans_orders.h
│   ├── gaussian_copula_factor.cpp # 复杂因子示例：高斯Copula模型
│   ├── gaussian_copula_factor.h
│   ├── bridge/
│   │   └── ingress.cpp        # 桥接层实现 - 数据路由和分发
│   └── utils/                 # 工具实现（模块化）
│       ├── trading_time.cpp   # 交易时间逻辑实现
│       ├── math_utils.cpp     # 数学工具实现
│       ├── nms_bucket_aggregator.cpp # 时间桶聚合器实现
│       ├── data_adapter.cpp   # 数据转换逻辑
│       └── log.cpp            # 日志系统实现
├── tests/                     # 测试代码
│   ├── factor_compute_test.cpp    # 测试主入口
│   ├── gaussian_copula_factor_test.cpp # 复杂因子测试
│   ├── tick_trans_orders_test.cpp # 基础因子测试
│   ├── test_wait.cpp          # 数据总线等待功能测试
│   └── utils/data_gen.h       # 测试数据生成工具
├── docs/                      # 文档
│   ├── CONFIRMED.md
│   ├── Logging.md
│   ├── Manual.md
│   └── demo_wiring.md
└── demo_header/               # 使用示例（外部依赖）
    ├── AppDemo.cpp
    ├── AppDemo.h
    └── ...
```

### 🔧 核心组件深度解析

#### 1. 因子接口 (IFactor) - 统一的因子契约

所有因子都必须继承自 `IFactor` 接口，这确保了框架的一致性：

```cpp
class IFactor {
public:
    virtual ~IFactor() = default;
    
    // 核心数据处理接口 - 必须实现
    virtual void on_quote(const QuoteDepth& q) = 0;      // 处理行情数据
    virtual void on_transaction(const Transaction& t) = 0; // 处理成交数据
    virtual void on_entrust(const Entrust& e) = 0;       // 处理委托数据
    
    // 强制刷新接口 - 必须实现
    virtual bool force_flush(const std::string& code) = 0;
    
    // 元信息接口 - 必须实现
    virtual std::string get_name() const = 0;           // 因子标识
    virtual const std::vector<std::string>& get_codes() const = 0; // 监控代码
};
```

**基础因子抽象类** `BaseFactor` 提供了通用实现：

```cpp
class BaseFactor : public IFactor {
protected:
    std::vector<std::string> _codes;  // 监控的股票代码列表
    std::string _name;                // 因子名称

public:
    BaseFactor(const std::string& name, std::vector<std::string> codes)
        : _name(name), _codes(std::move(codes)) {}

    // 默认实现
    std::string get_name() const override { return _name; }
    const std::vector<std::string>& get_codes() const override { return _codes; }
};
```

#### 2. 数据总线 (DataBus) - 因子通信的神经系统

数据总线是框架的核心，提供类型安全的发布-订阅机制：

**核心特性：**
- **主题注册**：每个主题绑定特定数据类型
- **环形缓冲区**：保留历史数据，自动淘汰旧数据
- **多维度索引**：按 (主题, 代码, 时间戳) 三维索引
- **阻塞等待**：支持跨因子数据依赖的同步

**关键API：**

```cpp
class DataBus {
public:
    static DataBus& instance();  // 单例模式
    
    // 主题管理
    template<typename T>
    void register_topic(const std::string& topic, size_t capacity=120);
    
    // 数据发布
    template<typename T>
    void publish(const std::string& topic, const std::string& code, 
                 int64_t ts_ms, const T& value);
    
    // 数据消费
    template<typename T>
    bool get_latest(const std::string& topic, const std::string& code, 
                    T& out, int64_t* ts_ms=nullptr) const;
    
    // 订阅机制
    template<typename T>
    void subscribe(const std::string& topic, const std::string& code,
                   std::function<void(const std::string&, int64_t, const T&)> cb);
    
    // 阻塞等待（用于因子依赖）
    template<typename T>
    bool wait_for_time_exact(const std::string& topic, const std::string& code,
                             int64_t ts_ms, T& out, int64_t timeout_ms = 1000);
};
```

#### 3. 时间桶聚合器 (NmsBucketAggregator) - 时序对齐引擎

专门处理高频数据的时间对齐问题：

```cpp
class NmsBucketAggregator {
public:
    explicit NmsBucketAggregator(int64_t bucket_ms=1000);
    
    void on_quote(const QuoteDepth& q);      // 计算成交额/成交量增量
    void on_transaction(const Transaction& t); // 收集成交切片
    void on_entrust(const Entrust& e);       // 收集委托切片
    
    bool flush_if_crossed(int64_t now_ms, BucketOutputs& out); // 检查桶边界
    bool force_flush(BucketOutputs& out);    // 强制产出
};
```

**时间桶工作原理：**
```
时间轴: 09:30:00.000 ──── 09:30:01.000 ──── 09:30:02.000 ────→
桶划分:    桶1    │       桶2     │       桶3     │
数据流入:  Q1,T1 │ Q2,E1 │ Q3,T2 │ Q4    │ Q5,T3  │
          │      │       │       │       │        │
产出时机:        ↓产出桶1 ↓       ↓产出桶2 ↓       ↓产出桶3
```

#### 4. 模块化工具组件

**数据类型 (types.h)** - 纯数据结构定义
```cpp
// 标准化的数据结构，不包含业务逻辑
struct QuoteDepth, Transaction, Entrust, Bar, BucketOutputs;
```

**交易时间工具 (trading_time.h/cpp)** - 时段判断
```cpp
class TradingTime {
    static bool in_trading_session_ms(int64_t ms);
    static bool in_call_auction_ms(int64_t ms);
    static int64_t next_trading_session_start(int64_t current_ms);
};
```

**数学工具 (math_utils.h/cpp)** - 统计计算
```cpp
class MathUtils {
    static double mean(const std::vector<double>& data);
    static double stddev(const std::vector<double>& data);
    static double quantile(const std::vector<double>& data, double percentile);
    static double normal_quantile(double p);
};
```

#### 5. 数据适配器 (DataAdapter) - 格式转换层

将外部数据格式转换为内部标准格式：

```cpp
class DataAdapter {
public:
    // 外部快照 → 内部QuoteDepth
    static QuoteDepth from_snapshot_sh(const SnapshotStockSH& snapshot);
    static QuoteDepth from_snapshot_sz(const std_SnapshotStockSZ& snapshot);
    
    // 委托成交 → 内部Transaction/Entrust
    static Transaction from_ord_exec(const OrdAndExeInfo& ord_exec);
    static bool is_trade(const OrdAndExeInfo& x);
    static Transaction to_transaction(const OrdAndExeInfo& x);
    static Entrust to_entrust(const OrdAndExeInfo& x);
    
    // K线转换
    static Bar from_kline(const BasicandEnhanceKLine& k);
};
```

## 🚀 快速开始

### 环境要求

- **C++17** 兼容编译器 (GCC 7+, Clang 5+, MSVC 2019+)
- **CMake** 3.15+
- **第三方依赖**:
   - Eigen3 (线性代数计算)
   - GoogleTest (测试框架，可选)
   - spdlog (日志系统，可选)

### 构建项目

```bash
# 克隆项目
git clone <repository-url>
cd factors_lib

# 创建构建目录
mkdir build && cd build

# 配置项目（确保第三方库已放置在 third_party/ 目录）
cmake ..

# 编译
make -j$(nproc)

# 运行测试
./run_tests
```

### 基本使用示例

```cpp
#include "factorlib/bridge/ingress.h"
#include "basic_factors/tick_trans_orders.h"
#include "utils/types.h"
#include "utils/trading_time.h"

// 初始化因子系统
void initialize_factor_system() {
    // 1. 注册数据总线主题（必须在创建因子前调用）
    TickTransOrders::register_topics(120); // 120条历史数据容量
    
    // 2. 创建因子配置
    TickTransOrdersConfig config;
    config.bucket_size_ms = 1000;      // 1秒时间桶
    config.emit_tick_interval = true;  // 发布tick间切片数据
    
    // 3. 创建因子实例
    std::vector<std::string> monitor_codes = {"000001.SZ", "000002.SZ"};
    auto factor = std::make_shared<TickTransOrders>(config, monitor_codes);
    
    // 4. 注册到系统
    std::vector<std::shared_ptr<factorlib::IFactor>> factors = {factor};
    factorlib::bridge::set_factors(factors);
}

// 在数据回调中喂入数据
void on_market_data(const std::vector<SnapshotStockSH>& snapshots) {
    factorlib::bridge::ingest_snapshot(snapshots);
}

void on_order_data(const std::vector<OrdAndExeInfo>& orders) {
    factorlib::bridge::ingest_ont(orders);
}
```

## 📈 如何添加新因子

### 步骤1：确定因子类型

#### A. 基础因子 - 直接从原始数据计算

**特征：**
- 直接消费行情、成交、委托等原始数据
- 不依赖其他因子的输出
- 通常包含时间桶聚合逻辑
- 计算结果发布到数据总线

**模板：**

```cpp
// my_basic_factor.h
#pragma once
#include "ifactor.h"
#include "utils/databus.h"
#include "utils/types.h"
#include "utils/nms_bucket_aggregator.h"

namespace factorlib {

class MyBasicFactor : public BaseFactor {
public:
    explicit MyBasicFactor(const std::string& name, std::vector<std::string> codes)
        : BaseFactor(name, std::move(codes)) {}
    
    // 实现数据处理接口
    void on_quote(const QuoteDepth& q) override;
    void on_transaction(const Transaction& t) override;
    void on_entrust(const Entrust& e) override;
    
    bool force_flush(const std::string& code) override;
    
    // 注册数据总线主题（静态方法）
    static void register_topics(size_t capacity = 120);
    
private:
    // 因子内部状态
    std::unordered_map<std::string, NmsBucketAggregator> _aggregators;
    std::unordered_map<std::string, double> _last_prices;
    
    // 发布结果到数据总线
    void publish_results(const std::string& code, int64_t timestamp, double value);
    
    // 因子计算逻辑
    double calculate_factor(const BucketOutputs& output, const std::string& code);
};

} // namespace factorlib
```

#### B. 复杂因子 - 依赖其他因子的输出

**特征：**
- 消费其他因子在数据总线上发布的结果
- 实现复杂的多因子组合或模型
- 通过订阅机制获取输入数据
- 可能涉及机器学习或统计模型

**模板：**

```cpp
// my_complex_factor.h
#pragma once
#include "ifactor.h"
#include "utils/databus.h"
#include "utils/types.h"

namespace factorlib {

class MyComplexFactor : public BaseFactor {
public:
    explicit MyComplexFactor(std::vector<std::string> codes)
        : BaseFactor("MyComplexFactor", std::move(codes)) {
        setup_subscriptions();  // 构造函数中设置订阅
    }
    
    // 复杂因子可能不需要处理所有原始数据
    void on_quote(const QuoteDepth& q) override { /* 可选 */ }
    void on_transaction(const Transaction& t) override { /* 可选 */ }
    void on_entrust(const Entrust& e) override { /* 可选 */ }
    
    bool force_flush(const std::string& code) override;
    
    static void register_topics(size_t capacity = 120);
    
private:
    void setup_subscriptions();
    void on_dependency_update(const std::string& code, int64_t ts, const double& value);
    
    // 内部状态
    std::unordered_map<std::string, std::deque<double>> _input_window;
    std::unordered_map<std::string, double> _last_output;
};

} // namespace factorlib
```

### 步骤2：实现因子逻辑

#### 基础因子完整实现示例：

```cpp
// my_basic_factor.cpp
#include "my_basic_factor.h"
#include "utils/log.h"
#include "utils/math_utils.h"

namespace factorlib {

// 定义数据总线主题（使用层级命名）
static const char* TOPIC_MY_FACTOR = "custom/vwap_factor";

void MyBasicFactor::register_topics(size_t capacity) {
    DataBus::instance().register_topic<double>(TOPIC_MY_FACTOR, capacity);
}

void MyBasicFactor::on_quote(const QuoteDepth& q) {
    // 确保该代码的聚合器存在
    if (_aggregators.find(q.instrument_id) == _aggregators.end()) {
        _aggregators.emplace(q.instrument_id, NmsBucketAggregator(1000));
    }
    
    auto& agg = _aggregators[q.instrument_id];
    
    // 检查是否需要产出上一个时间桶
    BucketOutputs output;
    if (agg.flush_if_crossed(q.data_time_ms, output)) {
        // 计算因子值并发布
        double factor_value = calculate_factor(output, q.instrument_id);
        publish_results(q.instrument_id, output.bucket_end_ms, factor_value);
    }
    
    // 处理当前行情
    agg.on_quote(q);
    
    // 更新最后价格（用于其他计算）
    _last_prices[q.instrument_id] = (q.bid_price + q.ask_price) / 2.0;
}

void MyBasicFactor::on_transaction(const Transaction& t) {
    auto it = _aggregators.find(t.instrument_id);
    if (it != _aggregators.end()) {
        it->second.on_transaction(t);
    }
}

void MyBasicFactor::on_entrust(const Entrust& e) {
    auto it = _aggregators.find(e.instrument_id);
    if (it != _aggregators.end()) {
        it->second.on_entrust(e);
    }
}

bool MyBasicFactor::force_flush(const std::string& code) {
    auto it = _aggregators.find(code);
    if (it == _aggregators.end()) return false;
    
    BucketOutputs output;
    if (it->second.force_flush(output)) {
        double factor_value = calculate_factor(output, code);
        publish_results(code, output.bucket_end_ms, factor_value);
        return true;
    }
    return false;
}

void MyBasicFactor::publish_results(const std::string& code, int64_t timestamp, double value) {
    DataBus::instance().publish<double>(TOPIC_MY_FACTOR, code, timestamp, value);
    LOG_DEBUG("[{}] 发布 {} @ {}: {:.6f}", get_name(), code, timestamp, value);
}

double MyBasicFactor::calculate_factor(const BucketOutputs& output, const std::string& code) {
    // 实现具体的因子计算逻辑
    // 示例：成交量加权平均价格 (VWAP)
    if (output.volume_sum == 0) return 0.0;
    return output.amount_sum / output.volume_sum;
}

} // namespace factorlib
```

#### 复杂因子完整实现示例：

```cpp
// my_complex_factor.cpp
#include "my_complex_factor.h"
#include "utils/log.h"
#include "utils/math_utils.h"
#include <numeric>

namespace factorlib {

static const char* TOPIC_COMPLEX_RESULT = "complex/moving_average";
static const char* TOPIC_DEPENDENCY = "custom/vwap_factor"; // 依赖的基础因子

void MyComplexFactor::register_topics(size_t capacity) {
    DataBus::instance().register_topic<double>(TOPIC_COMPLEX_RESULT, capacity);
}

void MyComplexFactor::setup_subscriptions() {
    auto& bus = DataBus::instance();
    
    for (const auto& code : get_codes()) {
        bus.subscribe<double>(TOPIC_DEPENDENCY, code,
            [this](const std::string& code, int64_t ts, const double& value) {
                this->on_dependency_update(code, ts, value);
            });
    }
}

void MyComplexFactor::on_dependency_update(const std::string& code, int64_t ts, const double& value) {
    // 维护滑动窗口
    auto& window = _input_window[code];
    window.push_back(value);
    
    // 保持窗口大小（示例：5个周期）
    const size_t window_size = 5;
    if (window.size() > window_size) {
        window.pop_front();
    }
    
    // 计算移动平均
    if (window.size() == window_size) {
        double sum = std::accumulate(window.begin(), window.end(), 0.0);
        double moving_avg = sum / window_size;
        
        _last_output[code] = moving_avg;
        DataBus::instance().publish<double>(TOPIC_COMPLEX_RESULT, code, ts, moving_avg);
        
        LOG_DEBUG("[{}] 计算 {} @ {}: 移动平均 = {:.6f}", 
                 get_name(), code, ts, moving_avg);
    }
}

bool MyComplexFactor::force_flush(const std::string& code) {
    // 复杂因子可能不需要特殊刷新逻辑，或者实现特定的刷新策略
    auto it = _last_output.find(code);
    if (it != _last_output.end()) {
        int64_t current_time = /* 获取当前时间 */;
        DataBus::instance().publish<double>(TOPIC_COMPLEX_RESULT, code, 
                                           current_time, it->second);
        LOG_INFO("[{}] 强制刷新 {}", get_name(), code);
        return true;
    }
    return false;
}

} // namespace factorlib
```

### 步骤3：注册和使用新因子

```cpp
#include "factorlib/bridge/ingress.h"
#include "my_basic_factor.h"
#include "my_complex_factor.h"
#include "utils/trading_time.h"

void setup_complete_factor_system() {
    // 1. 注册所有数据总线主题
    TickTransOrders::register_topics(120);
    MyBasicFactor::register_topics(120);
    MyComplexFactor::register_topics(120);
    
    // 2. 创建因子配置
    TickTransOrdersConfig tick_config{1000, true};
    std::vector<std::string> monitor_codes = {"000001.SZ", "000002.SZ", "000003.SZ"};
    
    // 3. 创建因子实例
    std::vector<std::shared_ptr<factorlib::IFactor>> factors;
    
    // 基础因子
    factors.push_back(std::make_shared<TickTransOrders>(tick_config, monitor_codes));
    factors.push_back(std::make_shared<MyBasicFactor>("MyVWAP", monitor_codes));
    
    // 复杂因子（依赖基础因子的输出）
    factors.push_back(std::make_shared<MyComplexFactor>(monitor_codes));
    
    // 4. 注册到系统
    factorlib::bridge::set_factors(factors);
    
    LOG_INFO("因子系统初始化完成，共注册 {} 个因子", factors.size());
}
```

## 🔄 完整数据处理流程

### 数据流架构图

```
外部数据源
     ↓
[ingress桥接层] 
     ↓ 数据格式转换 (DataAdapter)
因子管理器 (FactorManager) 
     ↓ 数据分发
各因子并行处理
     ↓
[基础因子] → 计算原始特征 → 发布到 DataBus
     ↓
[复杂因子] ← 订阅基础因子 → 计算组合特征 → 发布到 DataBus
     ↓
最终结果供策略消费
```

### 时序处理机制

框架通过严格的时间桶机制确保数据一致性：

1. **原始数据流入**：高频的行情、成交、委托数据
2. **时间桶划分**：按配置的时间窗口（如1000ms）划分数据
3. **桶内聚合**：在每个时间桶内累计成交量、成交额，收集切片数据
4. **桶结束触发**：当时间跨越桶边界时，自动产出该时间窗口的聚合结果
5. **时间戳对齐**：所有因子使用相同的桶结束时间作为发布时间戳，确保时序一致

**时间对齐示例：**
```
时间轴: 09:30:00.000 ──── 09:30:01.000 ──── 09:30:02.000 ────→
桶划分:    桶1      │       桶2       │       桶3       │
           ↓       ↓        ↓        ↓        ↓        ↓
因子A产出: 值A1 @ 09:30:01.000 │ 值A2 @ 09:30:02.000 │ ...
因子B产出: 值B1 @ 09:30:01.000 │ 值B2 @ 09:30:02.000 │ ...
```

## 🧪 测试策略

### 单元测试框架

项目使用 GoogleTest 框架，提供中文输出的测试结果：

```cpp
// 为每个因子创建对应的测试文件
#include <gtest/gtest.h>
#include "my_basic_factor.h"
#include "utils/data_gen.h"

TEST(MyFactorTest, BasicCalculation) {
    // 创建因子实例
    MyBasicFactor factor("TestFactor", {"TEST001"});
    
    // 生成测试数据
    auto test_series = factorlib::testutil::make_series_basic("TEST001", 
        factorlib::testutil::hms_ms(9, 30, 0, 0), 1000);
    
    // 执行测试
    for (const auto& quote : test_series.quotes) {
        factor.on_quote(quote);
    }
    
    // 验证结果
    // ...
}
```

### 测试数据生成工具

使用 `data_gen.h` 中的工具生成可预期的测试数据：

```cpp
#include "tests/utils/data_gen.h"

TEST(MyFactorTest, WithGeneratedData) {
    auto test_series = factorlib::testutil::make_series_basic("TEST001", 
        factorlib::testutil::hms_ms(9, 30, 0, 0), 1000);
    
    // 使用生成的测试数据喂入因子
    for (const auto& quote : test_series.quotes) {
        factor.on_quote(quote);
    }
    
    // 验证产出结果
}
```

## 📊 实际案例：高斯Copula因子

项目包含一个完整的高斯Copula因子实现，展示了复杂统计模型的应用：

### 算法原理

1. **特征提取**：订单流不平衡(OFI)和成交量作为输入特征
2. **秩转换**：将原始数据转换为均匀分布的秩统计量
3. **高斯转换**：使用逆正态CDF转换为多元正态分布
4. **条件期望**：基于多元正态分布计算条件期望
5. **逆转换**：将结果转换回原始收益率的尺度

### 实现亮点

```cpp
double GaussianCopulaFactor::compute_conditional_expectation(const std::string& code) {
    // 1. 秩转换和正态分位数转换
    std::vector<double> z_ofi(n), z_volume(n), z_return(n);
    
    // 2. 计算协方差矩阵（带正则化）
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    
    // 3. 分割协方差矩阵并计算条件均值
    Eigen::Vector2d beta = sigma_11.ldlt().solve(sigma_12);
    double conditional_mean = mean(2) + beta.dot(x_condition);
    
    // 4. 转换回均匀分布并应用经验逆CDF
    double conditional_probability = 0.5 * (1.0 + std::erf(conditional_mean / std::sqrt(2.0)));
    double predicted_return = empirical_inverse_cdf(state.return_window, conditional_probability);
    
    return predicted_return;
}
```

## 🔧 最佳实践和性能优化

### 命名规范

- **因子类名**：使用驼峰命名，如 `VolumeWeightedAveragePrice`
- **数据总线主题**：使用层级命名，如 `namespace/factor_name`
- **配置文件**：使用嵌套结构，明确参数含义
- **日志标识**：在日志中明确标识因子名称和股票代码

### 错误处理

```cpp
void MyFactor::on_quote(const QuoteDepth& q) {
    try {
        // 因子核心逻辑
        process_quote_data(q);
    } catch (const std::exception& e) {
        LOG_ERROR("[{}] 处理 {} 行情数据时出错: {}", 
                 get_name(), q.instrument_id, e.what());
        // 可以考虑重置状态或采取其他恢复措施
    }
}
```

### 性能考虑

1. **避免数据拷贝**：在数据总线中传递 const 引用
2. **合理设置容量**：根据内存和使用场景设置数据总线的环形缓冲区容量
3. **及时清理状态**：在 `force_flush` 中清理不必要的状态
4. **预分配内存**：对于频繁使用的数据结构，考虑预分配策略

### 内存管理

```cpp
class OptimizedFactor : public BaseFactor {
private:
    // 使用预分配的内存池避免频繁分配
    std::unordered_map<std::string, std::vector<double>> _preallocated_buffers;
    
    void preallocate_buffers() {
        for (const auto& code : get_codes()) {
            _preallocated_buffers[code].reserve(1000); // 预分配容量
        }
    }
};
```

## 🐛 故障排除和调试

### 常见问题

1. **数据总线主题未注册**
   ```cpp
   // 错误：在创建因子前忘记注册主题
   // 正确：在main函数或初始化时先注册主题
   MyFactor::register_topics(120);
   auto factor = std::make_shared<MyFactor>(...);
   ```

2. **时间戳不对齐**
   - 确保所有因子使用相同的桶结束时间作为发布时间戳
   - 检查时间桶配置是否一致

3. **内存泄漏**
   - 使用智能指针管理因子生命周期
   - 定期检查环形缓冲区容量

4. **性能问题**
   - 检查数据总线容量设置
   - 分析因子计算复杂度
   - 使用性能分析工具定位瓶颈

### 调试技巧

```cpp
// 启用详细日志
#define USE_SPDLOG
#include "utils/log.h"

// 在因子中添加调试输出
void MyFactor::on_quote(const QuoteDepth& q) {
    LOG_TRACE("[{}] 处理 {} @ {}", get_name(), q.instrument_id, q.data_time_ms);
    // ... 因子逻辑
}
```

## 🔮 扩展指南

### 添加新的数据源

1. 在 `DataAdapter` 中添加新的转换函数
2. 在 `ingress.h` 中添加新的数据摄入接口
3. 更新桥接层实现

### 自定义聚合逻辑

继承 `NmsBucketAggregator` 实现自定义时间桶策略：

```cpp
class CustomAggregator : public NmsBucketAggregator {
public:
    explicit CustomAggregator(int64_t bucket_ms) : NmsBucketAggregator(bucket_ms) {}
    
    void on_quote(const QuoteDepth& q) override {
        // 自定义聚合逻辑
        // ...
        
        // 调用基类实现（可选）
        NmsBucketAggregator::on_quote(q);
    }
};
```

## 📞 支持和贡献

### 获取帮助

- 查看 `docs/` 目录下的详细文档
- 参考现有因子实现作为模板
- 运行测试用例理解框架行为

### 贡献指南

1. 为每个新因子创建对应的测试用例
2. 遵循现有的代码风格和命名规范
3. 更新相关文档
4. 确保所有测试通过

### 许可证

[在此添加项目许可证信息]

---

## 🎯 架构优势总结

经过模块化重构后的 FactorLib 具有以下优势：

1. **清晰的职责分离**：每个组件都有明确的单一职责
2. **更好的编译时依赖**：只包含实际需要的头文件
3. **更高的可测试性**：可以独立测试各个工具组件
4. **更易维护**：相关功能集中管理，便于理解和修改
5. **更好的扩展性**：新增功能时可以放在合适的模块中
