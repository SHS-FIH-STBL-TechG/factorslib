# FactorLib - 量化因子计算框架

## 📖 项目概述
FactorLib 是一个**专业级量化因子计算框架**，专为高频量化交易场景设计。它提供一个标准化的因子开发、管理和数据流处理平台，让量化研究员专注于因子逻辑，而不必担心数据流、时间对齐和依赖管理等复杂工程问题。

### 🎯 核心设计理念
1. **统一接口规范**：所有因子都实现统一的 `IFactor` 接口，确保一致的开发体验。
2. **数据总线机制**：通过 `DataBus` 实现因子间的数据共享和依赖管理。
3. **时间桶聚合**：自动处理高频数据的时序对齐，确保计算一致性。
4. **最小入侵设计**：通过桥接层与外部系统解耦，易于集成到现有系统。
5. **类型安全**：模板化的数据总线确保数据类型安全。

---

## 🏗️ 项目架构详解

### 目录结构说明
```text
factors_lib/
├── include/                            # 公共头文件（对外接口）
│   ├── factor_manager.h                # 因子管理器 - 统一管理所有因子实例
│   ├── ifactor.h                       # 因子接口定义 - 所有因子的基类
│   ├── factorlib/bridge/
│   │   └── ingress.h                   # 数据入口桥接层 - 最小化外部依赖
│   └── utils/                          # 工具库
│       ├── data_adapter.h              # 数据格式适配器 - 外部数据 → 内部标准格式
│       ├── databus.h                   # 数据总线（核心） - 因子间通信枢纽
│       ├── log.h                       # 日志系统 - 统一日志接口
│       └── utils.h                     # 公共数据类型 - 标准数据结构和工具
├── src/                                # 实现文件
│   ├── basic_factors/                  # 基础因子实现
│   │   ├── tick_trans_orders.cpp       # 基础因子示例：时间桶聚合
│   │   └── tick_trans_orders.h
│   ├── gaussian_copula_factor.cpp      # 复杂因子示例：高斯 Copula 模型
│   ├── gaussian_copula_factor.h
│   ├── bridge/
│   │   └── ingress.cpp                 # 桥接层实现 - 数据路由和分发
│   └── utils/                          # 工具实现
│       ├── data_adapter.cpp            # 数据转换逻辑
│       ├── log.cpp
│       └── utils.cpp                   # 时间桶聚合器实现
├── tests/                              # 测试代码
│   ├── factor_compute_test.cpp         # 测试主入口
│   ├── gaussian_copula_factor_test.cpp # 复杂因子测试
│   ├── tick_trans_orders_test.cpp      # 基础因子测试
│   ├── test_wait.cpp                   # 数据总线等待功能测试
│   └── utils/data_gen.h                # 测试数据生成工具
├── docs/                               # 文档
│   ├── CONFIRMED.md
│   ├── Logging.md
│   ├── Manual.md
│   └── demo_wiring.md
└── demo_header/                        # 使用示例（外部依赖）
    ├── AppDemo.cpp
    ├── AppDemo.h
    └── ...
```

---

## 🔧 核心组件深度解析

### 1. 因子接口（IFactor）— 统一因子契约
所有因子都必须继承自 `IFactor` 接口，确保框架一致性。

```cpp
class IFactor {
public:
    virtual ~IFactor() = default;

    // 核心数据处理接口 - 必须实现
    virtual void on_quote(const QuoteDepth& q) = 0;        // 处理行情数据
    virtual void on_transaction(const Transaction& t) = 0; // 处理成交数据
    virtual void on_entrust(const Entrust& e) = 0;         // 处理委托数据

    // 强制刷新接口 - 必须实现
    virtual bool force_flush(const std::string& code) = 0;

    // 元信息接口 - 必须实现
    virtual std::string get_name() const = 0;                   // 因子标识
    virtual const std::vector<std::string>& get_codes() const = 0; // 监控代码
};
```

**基础因子抽象类** `BaseFactor` 提供通用实现：
```cpp
class BaseFactor : public IFactor {
protected:
    std::vector<std::string> _codes; // 监控的股票代码列表
    std::string _name;               // 因子名称

public:
    BaseFactor(const std::string& name, std::vector<std::string> codes)
        : _name(name), _codes(std::move(codes)) {}

    std::string get_name() const override { return _name; }
    const std::vector<std::string>& get_codes() const override { return _codes; }
};
```

### 2. 数据总线（DataBus）— 因子通信的神经系统
提供**类型安全**的发布-订阅机制：

**核心特性：** 主题注册、环形缓冲、(主题, 代码, 时间戳) 三维索引、阻塞等待。

```cpp
class DataBus {
public:
    static DataBus& instance(); // 单例

    // 主题管理
    template<typename T>
    void register_topic(const std::string& topic, size_t capacity = 120);

    // 数据发布
    template<typename T>
    void publish(const std::string& topic, const std::string& code, int64_t ts_ms, const T& value);

    // 最新值读取
    template<typename T>
    bool get_latest(const std::string& topic, const std::string& code, T& out, int64_t* ts_ms = nullptr) const;

    // 订阅
    template<typename T>
    void subscribe(const std::string& topic, const std::string& code,
                   std::function<void(const std::string&, int64_t, const T&)> cb);

    // 阻塞等待（用于因子依赖）
    template<typename T>
    bool wait_for_time_exact(const std::string& topic, const std::string& code,
                             int64_t ts_ms, T& out, int64_t timeout_ms = 1000);
};
```

### 3. 时间桶聚合器（NmsBucketAggregator）— 时序对齐引擎
专门处理高频数据的时间对齐。

```cpp
class NmsBucketAggregator {
public:
    explicit NmsBucketAggregator(int64_t bucket_ms = 1000);
    void on_quote(const QuoteDepth& q);
    void on_transaction(const Transaction& t);
    void on_entrust(const Entrust& e);
    bool flush_if_crossed(int64_t now_ms, BucketOutputs& out);
    bool force_flush(BucketOutputs& out);
};
```

**工作原理：**
```text
时间轴: 09:30:00.000 ─── 09:30:01.000 ─── 09:30:02.000 ──→
桶划分:   桶1          │     桶2          │     桶3
数据流:   Q1,T1        │     Q2,E1        │     Q3,T2,Q4,T3...
产出:             ↓产出桶1        ↓产出桶2        ↓产出桶3
```

### 4. 数据适配器（DataAdapter）— 格式转换层
将外部数据转换为内部标准格式：

```cpp
class DataAdapter {
public:
    // 外部快照 → 内部 QuoteDepth
    static QuoteDepth from_snapshot_sh(const SnapshotStockSH& snapshot);
    static QuoteDepth from_snapshot_sz(const std_SnapshotStockSZ& snapshot);

    // 委托/成交 → 内部 Transaction / Entrust
    static bool is_trade(const OrdAndExeInfo& x);
    static Transaction from_ord_exec(const OrdAndExeInfo& ord_exec);
    static Transaction to_transaction(const OrdAndExeInfo& x);
    static Entrust to_entrust(const OrdAndExeInfo& x);

    // K 线转换
    static Bar from_kline(const BasicandEnhanceKLine& k);
};
```

---

## 🚀 快速开始

### 环境要求
- **C++17** 兼容编译器（GCC 7+ / Clang 5+ / MSVC 2019+）
- **CMake** 3.15+
- **第三方依赖**：Eigen3（线性代数）、GoogleTest（测试，可选）、spdlog（日志，可选）

### 构建项目
```bash
# 克隆项目
git clone <repository-url>
cd factors_lib

# 创建构建目录
mkdir build && cd build

# 配置项目（确保第三方库位于 third_party/）
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

// 初始化因子系统
void initialize_factor_system() {
    // 1) 注册数据总线主题（在创建因子前调用）
    TickTransOrders::register_topics(120);

    // 2) 配置
    TickTransOrdersConfig config;
    config.bucket_size_ms = 1000;      // 1 秒时间桶
    config.emit_tick_interval = true;  // 发布 tick 间切片数据

    // 3) 创建因子实例
    std::vector<std::string> monitor_codes = {"000001.SZ", "000002.SZ"};
    auto factor = std::make_shared<TickTransOrders>(config, monitor_codes);

    // 4) 注册到系统
    std::vector<std::shared_ptr<factorlib::IFactor>> factors = { factor };
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

---

## 📈 如何添加新因子

### 步骤 1：确定因子类型

#### A. 基础因子（直接从原始数据计算）
**特征**：直接消费行情/成交/委托；不依赖其他因子；常含时间桶聚合；结果发布至 DataBus。

**模板**：
```cpp
// my_basic_factor.h
#pragma once
#include "ifactor.h"
#include "utils/databus.h"
#include "utils/utils.h"

namespace factorlib {
class MyBasicFactor : public BaseFactor {
public:
    explicit MyBasicFactor(const std::string& name, std::vector<std::string> codes)
        : BaseFactor(name, std::move(codes)) {}

    // 数据处理接口
    void on_quote(const QuoteDepth& q) override;
    void on_transaction(const Transaction& t) override;
    void on_entrust(const Entrust& e) override;
    bool force_flush(const std::string& code) override;

    // 注册 DataBus 主题
    static void register_topics(size_t capacity = 120);

private:
    std::unordered_map<std::string, NmsBucketAggregator> _aggregators;
    std::unordered_map<std::string, double> _last_prices;

    void publish_results(const std::string& code, int64_t timestamp, double value);
    double calculate_factor(const BucketOutputs& output, const std::string& code);
};
} // namespace factorlib
```

#### B. 复杂因子（依赖其他因子输出）
**特征**：订阅 DataBus 上其他因子的结果；进行组合/模型计算；再发布结果。

**模板**：
```cpp
// my_complex_factor.h
#pragma once
#include "ifactor.h"
#include "utils/databus.h"

namespace factorlib {
class MyComplexFactor : public BaseFactor {
public:
    explicit MyComplexFactor(std::vector<std::string> codes)
        : BaseFactor("MyComplexFactor", std::move(codes)) {
        setup_subscriptions();
    }

    void on_quote(const QuoteDepth& q) override { /* 可选 */ }
    void on_transaction(const Transaction& t) override { /* 可选 */ }
    void on_entrust(const Entrust& e) override { /* 可选 */ }
    bool force_flush(const std::string& code) override;

    static void register_topics(size_t capacity = 120);

private:
    void setup_subscriptions();
    void on_dependency_update(const std::string& code, int64_t ts, const double& value);

    std::unordered_map<std::string, std::deque<double>> _input_window;
    std::unordered_map<std::string, double> _last_output;
};
} // namespace factorlib
```

### 步骤 2：实现因子逻辑

**基础因子实现示例（节选）**：
```cpp
// my_basic_factor.cpp
#include "my_basic_factor.h"
#include "utils/log.h"

namespace factorlib {
static const char* TOPIC_MY_FACTOR = "custom/vwap_factor";

void MyBasicFactor::register_topics(size_t capacity) {
    DataBus::instance().register_topic<double>(TOPIC_MY_FACTOR, capacity);
}

void MyBasicFactor::on_quote(const QuoteDepth& q) {
    if (_aggregators.find(q.instrument_id) == _aggregators.end()) {
        _aggregators.emplace(q.instrument_id, NmsBucketAggregator(1000));
    }
    auto& agg = _aggregators[q.instrument_id];

    BucketOutputs output;
    if (agg.flush_if_crossed(q.data_time_ms, output)) {
        double v = calculate_factor(output, q.instrument_id);
        publish_results(q.instrument_id, output.bucket_end_ms, v);
    }
    agg.on_quote(q);
    _last_prices[q.instrument_id] = (q.bid_price + q.ask_price) / 2.0;
}

bool MyBasicFactor::force_flush(const std::string& code) {
    auto it = _aggregators.find(code);
    if (it == _aggregators.end()) return false;
    BucketOutputs output;
    if (it->second.force_flush(output)) {
        double v = calculate_factor(output, code);
        publish_results(code, output.bucket_end_ms, v);
        return true;
    }
    return false;
}
} // namespace factorlib
```

**复杂因子实现示例（节选）**：
```cpp
// my_complex_factor.cpp
#include "my_complex_factor.h"
#include "utils/log.h"
#include <numeric>

namespace factorlib {
static const char* TOPIC_COMPLEX_RESULT = "complex/moving_average";
static const char* TOPIC_DEPENDENCY    = "custom/vwap_factor";

void MyComplexFactor::register_topics(size_t capacity) {
    DataBus::instance().register_topic<double>(TOPIC_COMPLEX_RESULT, capacity);
}

void MyComplexFactor::setup_subscriptions() {
    auto& bus = DataBus::instance();
    for (const auto& code : get_codes()) {
        bus.subscribe<double>(TOPIC_DEPENDENCY, code,
            [this](const std::string& code, int64_t ts, const double& value){
                this->on_dependency_update(code, ts, value);
            });
    }
}
} // namespace factorlib
```

### 步骤 3：注册与使用新因子
```cpp
#include "factorlib/bridge/ingress.h"
#include "my_basic_factor.h"
#include "my_complex_factor.h"

void setup_complete_factor_system() {
    TickTransOrders::register_topics(120);
    MyBasicFactor::register_topics(120);
    MyComplexFactor::register_topics(120);

    TickTransOrdersConfig tick_config{1000, true};
    std::vector<std::string> monitor_codes = {"000001.SZ","000002.SZ","000003.SZ"};

    std::vector<std::shared_ptr<factorlib::IFactor>> factors;
    factors.push_back(std::make_shared<TickTransOrders>(tick_config, monitor_codes));
    factors.push_back(std::make_shared<MyBasicFactor>("MyVWAP", monitor_codes));
    factors.push_back(std::make_shared<MyComplexFactor>(monitor_codes));

    factorlib::bridge::set_factors(factors);
}
```

---

## 🔄 完整数据处理流程

### 数据流架构图
```text
外部数据源
        ↓
[ ingress 桥接层 ]
        ↓
数据格式转换 (DataAdapter)
        ↓
因子（IFactor）并行处理
        ↓
[ 基础因子 ] → 计算原始特征 → 发布到 DataBus
        ↓
[ 复杂因子 ] ← 订阅基础因子 → 组合特征 → 发布到 DataBus
        ↓
策略/下游消费
```

### 时序处理机制
1. **原始数据流入**：高频的行情/成交/委托数据。
2. **时间桶划分**：按配置的时间窗口（如 1000ms）划分。
3. **桶内聚合**：每桶累计成交量/成交额，收集切片。
4. **桶结束触发**：跨越桶边界产生聚合结果。
5. **时间戳对齐**：统一用“桶结束时间”作为发布时间戳，确保跨因子一致。

---

## 🧪 测试策略

### 单元测试框架
使用 GoogleTest，并已提供**中文输出**：
```cpp
TEST(MyFactorTest, BasicCalculation) {
    MyBasicFactor factor("TestFactor", {"TEST001"});
    QuoteDepth q; q.instrument_id = "TEST001"; q.data_time_ms = 1234567890000;
    q.volume = 1000; q.turnover = 10000.0;
    factor.on_quote(q);
    // 断言...
}
```

### 测试数据生成工具
```cpp
#include "tests/utils/data_gen.h"

TEST(MyFactorTest, WithGeneratedData) {
    auto series = factorlib::testutil::make_series_basic("TEST001", factorlib::testutil::hms_ms(9,30,0,0), 1000);
    for (const auto& q : series.quotes) {
        factor.on_quote(q);
    }
    // 断言...
}
```

---

## 📊 实际案例：高斯 Copula 因子

### 算法原理（要点）
1. **特征提取**：订单流不平衡（OFI）与成交量等。
2. **秩转换** → **正态分位数**：将原始数据映射到正态空间。
3. **协方差/正则化**：构造多元正态的协方差矩阵。
4. **条件期望**：在多元正态下计算条件均值。
5. **逆映射**：映回原收益尺度。

### 实现亮点（片段）
```cpp
double GaussianCopulaFactor::compute_conditional_expectation(const std::string& code) {
    // ... 计算协方差、条件均值 ...
    double conditional_probability = 0.5 * (1.0 + std::erf(conditional_mean / std::sqrt(2.0)));
    double predicted_return = empirical_inverse_cdf(state.return_window, conditional_probability);
    return predicted_return;
}
```

---

## 🔧 最佳实践与性能优化

### 命名规范
- 因子类名：驼峰式，如 `VolumeWeightedAveragePrice`。
- 主题命名：层级式，如 `namespace/factor_name`。
- 配置对象：结构化字段明确含义。
- 日志：含因子名与代码。

### 错误处理
```cpp
void MyFactor::on_quote(const QuoteDepth& q) {
    try {
        process_quote_data(q);
    } catch (const std::exception& e) {
        LOG_ERROR("[{}] 处理 {} 行情出错: {}", get_name(), q.instrument_id, e.what());
    }
}
```

### 性能建议
1. 避免不必要的数据拷贝。
2. 合理设置 DataBus 环形缓冲容量。
3. 在 `force_flush` 中清理状态。
4. 高频结构适当预分配。

### 内存管理示例
```cpp
class OptimizedFactor : public BaseFactor {
private:
    std::unordered_map<std::string, std::vector<double>> _preallocated_buffers;
    void preallocate_buffers() {
        for (const auto& code : get_codes()) {
            _preallocated_buffers[code].reserve(1000);
        }
    }
};
```

---

## 🐛 故障排除与调试

### 常见问题
1. **忘记注册主题**
   ```cpp
   // 错误：在创建因子前忘记注册主题
   // 正确：初始化时先注册
   MyFactor::register_topics(120);
   auto factor = std::make_shared<MyFactor>(...);
   ```
2. **时间戳不对齐**：统一使用“桶结束时间”。
3. **内存泄漏**：用智能指针管理生命周期，巡检环形缓冲容量。
4. **性能问题**：分析计算复杂度、容量设置，使用 profiler。

### 调试技巧
```cpp
#define USE_SPDLOG
#include "utils/log.h"

DataBus::instance().debug_print_topics();

void MyFactor::on_quote(const QuoteDepth& q) {
    LOG_TRACE("[{}] 处理 {} @ {}", get_name(), q.instrument_id, q.data_time_ms);
}
```

---

## 🔮 扩展指南

### 添加新的数据源
1. 在 `DataAdapter` 中添加新的转换函数。
2. 在 `ingress.h` 增加新的数据摄入接口。
3. 更新桥接层实现。

### 自定义聚合逻辑
```cpp
class CustomAggregator : public NmsBucketAggregator {
public:
    explicit CustomAggregator(int64_t bucket_ms) : NmsBucketAggregator(bucket_ms) {}
    void on_quote(const QuoteDepth& q) override {
        // 自定义逻辑 ...
        NmsBucketAggregator::on_quote(q);
    }
};
```

---

## 📞 支持与贡献

### 获取帮助
- 查看 `docs/` 目录下的详细文档。
- 参考现有因子实现作为模板。
- 运行测试用例理解框架行为。

### 贡献指南
1. 为每个新因子创建对应的测试用例。
2. 遵循现有代码风格和命名规范。
3. 更新相关文档。
4. 确保所有测试通过。

### 许可证
[在此添加项目许可证信息]
