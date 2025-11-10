# FactorLib - 量化因子计算框架

## 📋 项目概述

FactorLib 是一个专为金融量化分析设计的C++因子计算框架，提供高效、可扩展的因子实现和数据处理工具。框架采用模块化设计，支持从基础因子到复杂统计模型的全套计算流程。

### 🎯 核心设计理念

1. **统一接口规范**：通过 `IFactor` 基类确保所有因子遵循相同的接口契约
2. **数据驱动架构**：基于 `DataBus` 实现因子间的松耦合通信
3. **时序处理优化**：`NmsBucketAggregator` 提供精确的时间桶聚合
4. **增量计算**：支持滑动窗口统计量的高效更新
5. **多源数据适配**：通过 `DataAdapter` 统一不同数据源格式

## 🏗️ 项目架构详解

### 目录结构

```
factors_lib/
├── CMakeLists.txt              # 项目构建配置
├── include/                    # 公共头文件
│   ├── factorlib/bridge/
│   │   └── ingress.h          # 数据入口桥接接口
│   ├── ifactor.h              # 因子基类接口定义
│   └── utils/                 # 工具类头文件
│       ├── data_adapter.h     # 多源数据格式转换
│       ├── databus.h          # 数据总线通信系统
│       ├── log.h              # 分级日志系统
│       ├── math/              # 数学工具库
│       │   ├── distributions.h    # 概率分布计算
│       │   ├── incremental_rank.h # 增量排名算法
│       │   ├── linear_algebra.h   # 线性代数运算
│       │   ├── numeric_utils.h    # 数值工具函数
│       │   └── statistics.h       # 统计计算
│       ├── nms_bucket_aggregator.h # 时间桶聚合器
│       ├── trading_time.h     # 交易时间处理
│       └── types.h            # 统一数据类型定义
├── src/                       # 源文件实现
│   ├── basic_factors/         # 基础因子实现
│   │   ├── tick_trans_orders.cpp  # Tick数据转换因子
│   │   └── tick_trans_orders.h
│   ├── bridge/
│   │   └── ingress.cpp        # 数据入口实现
│   ├── gaussian_copula_factor.cpp # 高斯Copula因子
│   ├── gaussian_copula_factor.h
│   └── utils/                 # 工具类实现
│       ├── data_adapter.cpp   # 数据适配器
│       ├── log.cpp            # 日志系统
│       ├── nms_bucket_aggregator.cpp # 时间桶聚合
│       └── trading_time.cpp   # 交易时间
├── tests/                     # 测试代码
│   ├── factor_compute_test.cpp    # 因子计算测试
│   ├── gaussian_copula_factor_test.cpp # 高斯Copula测试
│   ├── gtest_printer_zh.h     # 中文测试输出
│   ├── test_wait.cpp          # 测试等待工具
│   ├── tick_trans_orders_test.cpp # Tick转换测试
│   └── utils/
│       └── data_gen.h         # 测试数据生成器
└── third_party/               # 第三方依赖
    ├── eigen/                 # Eigen线性代数库
    ├── googletest/            # GoogleTest测试框架
    └── spdlog/                # spdlog日志库
```

### 🎪 核心组件深度解析

#### 1. 因子接口 (IFactor) - 统一的因子契约

**设计目标**：为所有因子提供统一的接口规范，确保代码的一致性和可维护性。

**核心接口**：
```cpp
class IFactor {
public:
    virtual ~IFactor() = default;
    
    // 核心数据处理接口
    virtual void on_quote(const QuoteDepth& q) = 0;
    virtual void on_transaction(const Transaction& t) = 0;
    virtual void on_entrust(const Entrust& e) = 0;
    
    // 强制刷新接口
    virtual void on_bar(const Bar& b) {}
    virtual bool force_flush(const std::string& code) = 0;
    
    // 元数据接口
    virtual std::string get_name() const = 0;
    virtual const std::vector<std::string>& get_codes() const = 0;
};
```

**实现特点**：
- **事件驱动设计**：通过 `on_quote`、`on_transaction`、`on_entrust` 方法处理不同类型的数据
- **强制刷新机制**：`force_flush` 方法确保在收盘或特定时刻输出计算结果
- **多代码支持**：单个因子实例可以同时监控多个股票代码

**BaseFactor 基类**：
```cpp
class BaseFactor : public IFactor {
protected:
    std::vector<std::string> _codes;
    std::string _name;
    
public:
    BaseFactor(const std::string& name, std::vector<std::string> codes)
        : _name(name), _codes(std::move(codes)) {}
    
    // 提供默认实现
    std::string get_name() const override { return _name; }
    const std::vector<std::string>& get_codes() const override { return _codes; }
};
```

#### 2. 数据总线 (DataBus) - 因子通信的神经系统

**设计目标**：实现因子间的松耦合通信，支持类型安全的数据交换和时间对齐。

**核心特性**：
- **类型安全**：每个Topic绑定特定数据类型，编译期检查
- **时间对齐**：支持按精确时间戳读取数据
- **环形历史**：自动淘汰旧数据，内存可控
- **多种访问模式**：支持拉取、订阅、阻塞等待

**关键API详解**：

**注册Topic**：
```cpp
template<typename T>
void register_topic(const std::string& topic, size_t capacity=120);
```
- `topic`：层级主题名，如 `"zyd/amount"`
- `capacity`：环形缓冲区容量，控制内存使用

**发布数据**：
```cpp
template<typename T>
void publish(const std::string& topic, const std::string& code, 
             int64_t ts_ms, const T& value);
```
- `ts_ms`：建议使用"桶结束时间"确保时间对齐
- 自动唤醒等待该数据的订阅者

**数据读取模式**：
```cpp
// 1. 读取最新数据
bool get_latest(const std::string& topic, const std::string& code, 
                T& out, int64_t* ts_ms=nullptr);

// 2. 按精确时间戳读取
bool get_by_time_exact(const std::string& topic, const std::string& code, 
                       int64_t ts_ms, T& out);

// 3. 读取最近N条数据
std::vector<std::pair<int64_t, T>> get_last_n(const std::string& topic, 
                                              const std::string& code, size_t n);
```

**订阅机制**：
```cpp
template<typename T>
void subscribe(const std::string& topic, const std::string& code,
               std::function<void(const std::string&, int64_t, const T&)> cb);
```
- 发布数据时自动触发回调
- 支持同一个Topic的多个订阅者

**阻塞等待**：
```cpp
// 等待精确时间戳的数据
bool wait_for_time_exact(const std::string& topic, const std::string& code,
                         int64_t ts_ms, T& out, int64_t timeout_ms = 1000);

// 等待不早于指定时间戳的数据
bool wait_for_time_at_least(const std::string& topic, const std::string& code,
                            int64_t ts_ms, T& out, int64_t timeout_ms = 1000);
```

#### 3. 时间桶聚合器 (NmsBucketAggregator) - 时序对齐引擎

**设计目标**：将高频数据聚合到固定时间桶，为因子计算提供时间对齐的输入。

**核心算法**：
```cpp
class NmsBucketAggregator {
public:
    explicit NmsBucketAggregator(int64_t bucket_ms=1000);
    
    // 数据输入接口
    void on_quote(const QuoteDepth& q);      // 用于计算amount/volume和更新midprice
    void on_transaction(const Transaction& t); // 仅用于桶切片
    void on_entrust(const Entrust& e);       // 仅用于桶切片
    
    // 桶产出接口
    bool flush_if_crossed(int64_t now_ms, BucketOutputs& out);
    bool force_flush(BucketOutputs& out);
    bool ensure_bucket(int64_t ts_ms, BucketOutputs& out);
};
```

**聚合输出结构**：
```cpp
struct BucketOutputs {
    double amount_sum = 0.0;              // 成交额累计
    int64_t volume_sum = 0;               // 成交量累计
    double midprice_last = 0.0;           // 最后中间价
    std::vector<Transaction> trans;       // 桶内成交记录
    std::vector<Entrust> orders;          // 桶内委托记录
    int64_t bucket_start_ms = 0;          // 桶开始时间
    int64_t bucket_end_ms = 0;            // 桶结束时间
};
```

**增量计算优化**：
- 维护上一次行情状态，避免重复计算
- 支持跨交易日的volume/turnover重置
- 智能桶边界检测，确保数据完整性

#### 4. 模块化工具组件

##### 数据类型 (types.h)

**统一数据结构定义**：
```cpp
// L2行情数据
struct QuoteDepth {
    std::string instrument_id{};     // 合约代码
    int64_t data_time_ms{0};         // 数据时间戳(毫秒)
    int trading_day{0};              // 交易日(YYYYMMDD)
    uint64_t volume{0};              // 成交量
    double turnover{0.0};            // 成交额
    double bid_price{0.0};           // 买一价
    double ask_price{0.0};           // 卖一价
};

// 成交记录
struct Transaction {
    std::string instrument_id{};
    int64_t data_time_ms{0};
    uint64_t main_seq{0};            // 主推序号
    double price{0.0};               // 成交价格
    int side{0};                     // 买卖方向
    uint64_t volume{0};              // 成交数量
    uint64_t bid_no{0};              // 买方订单号
    uint64_t ask_no{0};              // 卖方订单号
};

// 委托记录  
struct Entrust {
    std::string instrument_id{};
    int64_t data_time_ms{0};
    uint64_t main_seq{0};
    double price{0.0};
    int side{0};                     // 1:买, -1:卖
    uint64_t volume{0};
    uint64_t order_id{0};            // 委托订单号
};
```

##### 交易时间工具 (trading_time.h/cpp)

**核心功能**：
```cpp
class TradingTime {
public:
    // A股日盘交易时段判断
    static bool in_trading_session_ms(int64_t ms);
    
    // 集合竞价时段判断
    static bool in_call_auction_ms(int64_t ms);
    
    // 获取下一个交易时段开始时间
    static int64_t next_trading_session_start(int64_t current_ms);
    
    // 有效交易日判断（排除周末和节假日）
    static bool is_valid_trading_day(int trading_day);
};
```

##### 数学工具 (math工具集)

**统计计算** (`statistics.h`)：
```cpp
template<typename T>
class Statistics {
public:
    // 基础统计量
    template<typename Container> static double mean(const Container& data);
    template<typename Container> static double stddev(const Container& data);
    template<typename Container> static double median(const Container& data);
    
    // 分位数计算
    template<typename Container> static double quantile(const Container& data, double percentile);
    
    // 相关性分析
    template<typename Container1, typename Container2> 
    static double correlation(const Container1& x, const Container2& y);
    
    // 滑动窗口统计
    template<typename Container> 
    static std::vector<double> rolling_mean(const Container& data, size_t window_size);
};
```

**增量排名计算** (`incremental_rank.h`)：
- O(log n) 时间复杂度的滑动窗口排名计算
- 支持中位秩、分位数等统计量
- 内存高效的排序维护

#### 5. 数据适配器 (DataAdapter) - 格式转换层

**多源数据支持**：
```cpp
class DataAdapter {
public:
    // 不同交易所快照数据转换
    static QuoteDepth from_snapshot_sh(const SnapshotStockSH& snapshot);
    static QuoteDepth from_snapshot_sz(const std_SnapshotStockSZ& snapshot);
    
    // 成交数据转换
    static Transaction from_ord_exec(const OrdAndExeInfo& ord_exec);
    
    // 逐笔数据拆分
    static bool is_trade(const OrdAndExeInfo& x);
    static Transaction to_transaction(const OrdAndExeInfo& x);
    static Entrust to_entrust(const OrdAndExeInfo& x);
    
    // 价格标准化
    static double normalize_price(uint32_t raw_price);
};
```

## 🚀 快速开始

### 环境要求

- **操作系统**：Linux / Windows / macOS
- **编译器**：支持 C++17 (GCC 7+, Clang 5+, MSVC 2019+)
- **构建工具**：CMake 3.10+
- **内存**：建议 8GB+
- **磁盘空间**：500MB+（包含第三方依赖）

### 构建项目

**标准构建流程**：
```bash
# 克隆项目（如使用版本控制）
git clone <repository-url>
cd factors_lib

# 创建构建目录
mkdir build && cd build

# 配置项目
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
make -j$(nproc)

# 运行测试
ctest --output-on-failure
```

**Windows构建**：
```cmd
# 使用Visual Studio
mkdir build && cd build
cmake .. -G "Visual Studio 16 2019" -A x64
cmake --build . --config Release
```

### 基本使用示例

**1. 初始化因子计算环境**：
```cpp
#include "factors_lib/include/ifactor.h"
#include "factors_lib/include/utils/databus.h"
#include "factors_lib/include/utils/log.h"

// 初始化日志系统
auto logger = factors_lib::utils::log::init_logger("MyFactorApp");
logger->info("开始因子计算");

// 注册数据总线主题
factors_lib::TickTransOrders::register_topics(120);
factors_lib::GaussianCopulaFactor::register_topics(60);
```

**2. 创建并运行基础因子**：
```cpp
// 配置Tick转换因子
factors_lib::TickTransOrdersConfig tick_cfg;
tick_cfg.bucket_size_ms = 1000;  // 1秒时间桶
tick_cfg.emit_tick_interval = true;

std::vector<std::string> codes = {"000001.SZ", "600000.SH"};
auto tick_factor = factors_lib::TickTransOrders(tick_cfg, codes);

// 处理行情数据
factors_lib::QuoteDepth quote;
quote.instrument_id = "000001.SZ";
quote.data_time_ms = 1704065400000;  // 2024-01-01 09:30:00
quote.bid_price = 10.0;
quote.ask_price = 10.2;
quote.volume = 1000;
quote.turnover = 10000.0;

tick_factor.on_quote(quote);
```

**3. 使用数据总线获取计算结果**：
```cpp
auto& bus = factors_lib::DataBus::instance();

// 读取最新成交额
double amount;
int64_t timestamp;
if (bus.get_latest<double>("zyd/amount", "000001.SZ", amount, &timestamp)) {
    logger->info("股票 {} 在 {} 的成交额: {}", "000001.SZ", timestamp, amount);
}

// 订阅数据更新
bus.subscribe<double>("zyd/amount", "000001.SZ", 
    [](const std::string& code, int64_t ts, const double& value) {
        std::cout << "实时成交额更新: " << code << " at " << ts 
                  << " = " << value << std::endl;
    });
```

## 🔧 如何添加新因子

### 步骤1：确定因子类型

#### A. 基础因子 - 直接从原始数据计算

**特征**：
- 直接处理 `QuoteDepth`、`Transaction`、`Entrust` 等原始数据
- 使用 `NmsBucketAggregator` 进行时间桶聚合
- 计算结果通过 `DataBus` 发布

**适用场景**：
- 成交量相关因子
- 价格动量因子
- 订单簿分析因子

#### B. 复杂因子 - 依赖其他因子的输出

**特征**：
- 通过 `DataBus` 订阅其他因子的计算结果
- 进行复杂的统计建模或机器学习
- 可能需要等待多个输入因子的时间对齐

**适用场景**：
- 相关性分析因子
- 风险模型因子
- 机器学习预测因子

### 步骤2：实现因子逻辑

#### 基础因子完整实现示例：

**头文件** (`my_custom_factor.h`)：
```cpp
#pragma once
#include "ifactor.h"
#include "utils/databus.h"
#include "utils/nms_bucket_aggregator.h"

namespace factorlib {

struct MyCustomFactorConfig {
    int64_t bucket_size_ms = 5000;  // 5秒时间桶
    double threshold = 0.1;         // 自定义阈值
};

class MyCustomFactor : public BaseFactor {
public:
    explicit MyCustomFactor(const MyCustomFactorConfig& cfg, 
                           std::vector<std::string> codes);
    
    // 注册输出主题
    static void register_topics(size_t capacity = 120);
    
    // 实现IFactor接口
    void on_quote(const QuoteDepth& q) override;
    void on_transaction(const Transaction& t) override;
    void on_entrust(const Entrust& e) override;
    bool force_flush(const std::string& code) override;

private:
    MyCustomFactorConfig _cfg;
    std::unordered_map<std::string, NmsBucketAggregator> _aggregators;
    
    void ensure_code(const std::string& code);
    void publish_results(const std::string& code, const BucketOutputs& out);
};

} // namespace factorlib
```

**实现文件** (`my_custom_factor.cpp`)：
```cpp
#include "my_custom_factor.h"

namespace factorlib {

// 主题定义
static const char* TOP_MY_FACTOR = "custom/my_factor";

MyCustomFactor::MyCustomFactor(const MyCustomFactorConfig& cfg, 
                               std::vector<std::string> codes)
    : BaseFactor("MyCustomFactor", std::move(codes)), _cfg(cfg) {}

void MyCustomFactor::register_topics(size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_MY_FACTOR, capacity);
}

void MyCustomFactor::on_quote(const QuoteDepth& q) {
    ensure_code(q.instrument_id);
    
    // 检查时间桶边界
    BucketOutputs out;
    if (_aggregators[q.instrument_id].flush_if_crossed(q.data_time_ms, out)) {
        publish_results(q.instrument_id, out);
    }
    
    // 处理当前行情
    _aggregators[q.instrument_id].on_quote(q);
    
    // 自定义计算逻辑
    double spread = q.ask_price - q.bid_price;
    if (spread > _cfg.threshold) {
        // 执行特定逻辑
    }
}

void MyCustomFactor::on_transaction(const Transaction& t) {
    ensure_code(t.instrument_id);
    _aggregators[t.instrument_id].on_transaction(t);
}

void MyCustomFactor::on_entrust(const Entrust& e) {
    ensure_code(e.instrument_id);
    _aggregators[e.instrument_id].on_entrust(e);
}

bool MyCustomFactor::force_flush(const std::string& code) {
    auto it = _aggregators.find(code);
    if (it == _aggregators.end()) return false;
    
    BucketOutputs out;
    if (it->second.force_flush(out)) {
        publish_results(code, out);
        return true;
    }
    return false;
}

void MyCustomFactor::ensure_code(const std::string& code) {
    if (_aggregators.find(code) == _aggregators.end()) {
        _aggregators.emplace(code, NmsBucketAggregator(_cfg.bucket_size_ms));
    }
}

void MyCustomFactor::publish_results(const std::string& code, const BucketOutputs& out) {
    auto& bus = DataBus::instance();
    
    // 自定义因子计算
    double factor_value = out.amount_sum / (out.volume_sum + 1e-6);
    
    bus.publish<double>(TOP_MY_FACTOR, code, out.bucket_end_ms, factor_value);
}

} // namespace factorlib
```

#### 复杂因子完整实现示例：

**头文件** (`my_complex_factor.h`)：
```cpp
#pragma once
#include "ifactor.h"
#include "utils/databus.h"
#include "utils/math/statistics.h"

namespace factorlib {

class MyComplexFactor : public BaseFactor {
public:
    explicit MyComplexFactor(std::vector<std::string> codes);
    
    static void register_topics(size_t capacity = 120);
    
    // 复杂因子通常不直接处理原始数据
    void on_quote(const QuoteDepth& q) override { /* 可选实现 */ }
    void on_transaction(const Transaction& t) override { /* 可选实现 */ }
    void on_entrust(const Entrust& e) override { /* 可选实现 */ }
    bool force_flush(const std::string& code) override;
    
    // 启动计算（通常在数据就绪后调用）
    void start_computation();

private:
    std::unordered_map<std::string, std::vector<double>> _input_data;
    
    void subscribe_inputs();
    void on_input_updated(const std::string& code, int64_t ts, const double& value);
    double compute_complex_value(const std::string& code);
};

} // namespace factorlib
```

### 步骤3：注册和使用新因子

**1. 在CMake中添加新因子**：
```cmake
# 在 factor_basic 库中添加新文件
add_library(factor_basic
    src/basic_factors/tick_trans_orders.cpp
    src/gaussian_copula_factor.cpp
    src/basic_factors/my_custom_factor.cpp  # 新增
    src/complex_factors/my_complex_factor.cpp  # 新增
)
```

**2. 集成到应用程序**：
```cpp
// 注册主题
factors_lib::MyCustomFactor::register_topics(100);
factors_lib::MyComplexFactor::register_topics(80);

// 创建因子实例
factors_lib::MyCustomFactorConfig custom_cfg;
custom_cfg.bucket_size_ms = 3000;
auto custom_factor = factors_lib::MyCustomFactor(custom_cfg, {"000001.SZ"});

// 使用因子
custom_factor.on_quote(some_quote_data);
```

## 📊 完整数据处理流程

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

**时间桶对齐策略**：
```cpp
// 确保所有因子使用相同的时间戳对齐
void process_data_with_time_alignment() {
    int64_t current_bucket_end = get_current_bucket_end();
    
    // 基础因子计算
    base_factor.on_quote(quote);
    
    // 复杂因子等待基础因子输出
    double base_output;
    if (bus.wait_for_time_exact("base/topic", "000001.SZ", 
                               current_bucket_end, base_output, 1000)) {
        // 时间对齐成功，进行计算
        complex_factor.compute(base_output);
    }
}
```

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

**测试代码结构**：
```cpp
#include <gtest/gtest.h>
#include "utils/data_gen.h"

class TickTransOrdersTest : public ::testing::Test {
protected:
    void SetUp() override {
        factors_lib::TickTransOrders::register_topics(50);
        _factor = std::make_unique<factors_lib::TickTransOrders>(_cfg, {"TEST001"});
    }
    
    factors_lib::TickTransOrdersConfig _cfg;
    std::unique_ptr<factors_lib::TickTransOrders> _factor;
};

TEST_F(TickTransOrdersTest, BasicAggregation) {
    // 使用测试数据生成器
    auto series = factors_lib::testutil::make_series_basic("TEST001", 
                                                          1704065400000, 1000);
    
    // 处理测试数据
    for (const auto& quote : series.quotes) {
        _factor->on_quote(quote);
    }
    
    // 验证结果
    double amount;
    ASSERT_TRUE(DataBus::instance().get_latest<double>("zyd/amount", 
                                                      "TEST001", amount));
    EXPECT_NEAR(amount, 200000.0, 1e-6);
}
```

### 测试数据生成工具

**`data_gen.h` 核心功能**：
```cpp
namespace factorlib::testutil {

// 时间转换工具
inline int64_t hms_ms(int H, int M, int S, int ms = 0) {
    return ((H * 3600LL + M * 60LL + S) * 1000LL + ms);
}

// 构造可预期的测试序列
Series make_series_basic(const std::string& code, int64_t start_ms, int64_t bucket_ms) {
    Series s;
    // 构造精确的行情、成交、委托序列
    // 确保每个时间桶的累计值可预测
    return s;
}

} // namespace
```

## 📈 实际案例：高斯Copula因子

### 算法原理

**高斯Copula条件期望**：
1. **数据预处理**：将OFI、成交量、收益率转换为正态分数
2. **协方差估计**：计算多变量高斯分布的协方差矩阵
3. **条件期望**：给定OFI和成交量，预测收益率的条件期望
4. **逆变换**：将正态分布结果转换回原始收益率尺度

**数学公式**：
```
Z_return | Z_ofi, Z_volume ~ N(μ_cond, Σ_cond)
其中：
μ_cond = μ_return + Σ_{return,[ofi,volume]} · Σ_{[ofi,volume]}^{-1} · (Z_obs - μ_obs)
```

### 实现亮点

**增量计算优化**：
```cpp
void IncrementalState::update_data(double ofi, double volume, double ret) {
    // 增量更新秩计算器
    ofi_rank_calc.push(ofi, window_size);
    volume_rank_calc.push(volume, window_size);
    return_rank_calc.push(ret, window_size);
    
    // 增量更新协方差
    if (is_window_full()) {
        double z_ofi = math::Distributions::normal_quantile(
            ofi_rank_calc.median_rank(ofi));
        double z_volume = math::Distributions::normal_quantile(
            volume_rank_calc.median_rank(volume));
        double z_return = math::Distributions::normal_quantile(
            return_rank_calc.median_rank(ret));
            
        Eigen::Vector3d normal_score(z_ofi, z_volume, z_return);
        cov_calc.push(normal_score);
    }
}
```

**性能优势**：
- **时间复杂度**：O(log n) 的滑动窗口更新
- **内存效率**：只维护必要的统计量，不存储完整窗口
- **数值稳定性**：正则化协方差矩阵避免奇异性

## ⚡ 最佳实践和性能优化

### 命名规范

**主题命名约定**：
```
{作者或团队}/{因子类型}/{具体指标}
示例：
zyd/amount           # 成交额
zyd/tick/trans       # Tick成交切片  
gaussian_copula/prediction  # 高斯Copula预测值
```

**代码命名**：
- 类名：`PascalCase`，如 `TickTransOrders`
- 变量名：`snake_case`，如 `bucket_size_ms`
- 常量：`UPPER_SNAKE_CASE`，如 `TOP_AMOUNT`

### 错误处理

**防御性编程**：
```cpp
void ensure_code(const std::string& code) {
    if (_aggregators.find(code) == _aggregators.end()) {
        // 延迟初始化
        _aggregators.emplace(code, NmsBucketAggregator(_cfg.bucket_size_ms));
        LOG_DEBUG("初始化代码 {} 的聚合器", code);
    }
}

bool safe_publish(const std::string& topic, const std::string& code, 
                  int64_t ts, const T& value) {
    try {
        DataBus::instance().publish(topic, code, ts, value);
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("发布数据失败: {} - {}", topic, e.what());
        return false;
    }
}
```

### 性能考虑

**内存管理**：
- 使用 `std::deque` 作为环形缓冲区，自动淘汰旧数据
- 合理设置 `capacity` 参数，平衡内存和历史深度需求
- 对于大窗口因子，考虑使用增量计算避免存储完整数据

**计算优化**：
```cpp
// 好的实践：使用引用避免拷贝
void process_large_data(const std::vector<QuoteDepth>& quotes) {
    for (const auto& quote : quotes) {  // 使用const引用
        factor.on_quote(quote);
    }
}

// 避免在热路径中分配内存
void on_quote(const QuoteDepth& q) override {
    // 预分配数据结构
    thread_local BucketOutputs out;
    
    if (_aggregator.flush_if_crossed(q.data_time_ms, out)) {
        publish_results(out);
        out = BucketOutputs{};  // 重置而不是重新分配
    }
}
```

### 内存管理

**智能指针使用**：
```cpp
class FactorManager {
private:
    std::vector<std::unique_ptr<IFactor>> _factors;
    
public:
    void add_factor(std::unique_ptr<IFactor> factor) {
        _factors.push_back(std::move(factor));
    }
    
    // 自动内存管理，无需手动delete
};
```

## 🔍 故障排除和调试

### 常见问题

**1. 数据总线主题未注册**：
```
错误：尝试访问未注册的主题 'unknown/topic'
解决：在因子构造函数中调用 register_topics()
```

**2. 时间桶未对齐**：
```
现象：复杂因子读取不到基础因子的输出
解决：确保使用相同的时间戳（桶结束时间）发布和订阅
```

**3. 内存泄漏**：
```
检测：使用Valgrind或AddressSanitizer
预防：合理设置DataBus的capacity参数
```

### 调试技巧

**日志分级**：
```cpp
// 在开发阶段使用详细日志
logger->set_level(spdlog::level::debug);

// 关键路径添加跟踪日志
LOG_TRACE("开始处理代码 {} 的行情数据", code);
factor.on_quote(quote);
LOG_TRACE("完成处理，当前时间桶: {}", current_bucket);
```

**数据验证**：
```cpp
void validate_quote(const QuoteDepth& q) {
    if (q.bid_price <= 0 || q.ask_price <= 0) {
        LOG_WARN("异常价格数据: bid={}, ask={}", q.bid_price, q.ask_price);
    }
    
    if (q.data_time_ms == 0) {
        LOG_ERROR("无效时间戳: {}", q.instrument_id);
        throw std::invalid_argument("时间戳不能为0");
    }
}
```

## 🔌 扩展指南

### 添加新的数据源

**实现新的DataAdapter**：
```cpp
class MyExchangeDataAdapter {
public:
    static QuoteDepth from_my_exchange_format(const MyExchangeSnapshot& snapshot) {
        QuoteDepth q;
        q.instrument_id = snapshot.symbol;
        q.data_time_ms = snapshot.timestamp;
        q.bid_price = snapshot.best_bid;
        q.ask_price = snapshot.best_ask;
        q.volume = snapshot.volume;
        q.turnover = snapshot.turnover;
        return q;
    }
};
```

### 自定义聚合逻辑

**扩展NmsBucketAggregator**：
```cpp
class MyCustomAggregator : public NmsBucketAggregator {
public:
    using NmsBucketAggregator::NmsBucketAggregator;
    
    void on_quote(const QuoteDepth& q) override {
        // 先调用基类实现
        NmsBucketAggregator::on_quote(q);
        
        // 添加自定义聚合逻辑
        _custom_metric += calculate_custom_value(q);
    }
    
private:
    double _custom_metric = 0.0;
};
```

## 🤝 支持和贡献

### 获取帮助

- **问题报告**：在GitHub Issues中描述遇到的问题
- **功能请求**：通过Issue模板提交新功能建议
- **技术讨论**：参与项目的Discussions板块

### 贡献指南

**代码提交流程**：
1. Fork项目仓库
2. 创建功能分支：`git checkout -b feature/amazing-feature`
3. 提交更改：`git commit -m 'Add amazing feature'`
4. 推送到分支：`git push origin feature/amazing-feature`
5. 创建Pull Request

**代码质量要求**：
- 通过所有现有测试
- 添加新功能的单元测试
- 更新相关文档
- 遵循项目的代码风格

### 许可证

本项目采用 **MIT 许可证**。详细信息请查看项目根目录中的 `LICENSE` 文件。

## 🏆 架构优势总结

1. **高性能**：增量计算和内存优化设计
2. **可扩展**：模块化架构支持快速添加新因子
3. **类型安全**：编译期检查确保数据一致性
4. **时间对齐**：精确的时间桶聚合机制
5. **松耦合**：数据总线实现因子间解耦
6. **生产就绪**：完整的错误处理和日志系统

---

**开始使用FactorLib**：
```bash
git clone https://github.com/your-username/factorlib.git
cd factorlib
mkdir build && cd build
cmake .. && make -j4
./tests/run_tests
```

