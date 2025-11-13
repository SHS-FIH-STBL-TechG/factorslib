# FactorLib - 量化因子计算框架

## 📋 项目概述

FactorLib 是一个专为金融量化分析设计的 C++ 因子计算框架，提供高效、可扩展的因子实现和数据处理工具。框架采用模块化设计，支持从基础因子到复杂统计模型的全套计算流程。

### 🎯 核心设计理念

1. **统一接口规范**：通过 `IFactor` 基类确保所有因子遵循相同的接口契约
2. **数据驱动架构**：基于 `DataBus` 实现因子间的松耦合通信
3. **时序处理优化**：`NmsBucketAggregator` 提供精确的时间桶聚合
4. **增量计算**：支持滑动窗口统计量的高效更新
5. **多源数据适配**：通过 `DataAdapter` 统一不同数据源格式

---

## 🏗️ 项目架构详解

### 目录结构

```
factors_lib/
├── CMakeLists.txt              # 项目构建配置
├── README.md
├── docs/                       # 额外文档与示例说明
│   ├── demo_wiring.md
│   └── µû░σó₧σ¢áσ¡Éσ╝ÇσÅæΣ╕ÄµÄÑσàÑµîçσìù.md
├── include/                    # 公共头文件
│   ├── factorlib/bridge/
│   │   └── ingress.h           # 数据入口桥接接口
│   ├── ifactor.h               # 因子基类接口定义（IFactor / BaseFactor）
│   └── utils/                  # 工具类头文件
│       ├── config/
│       │   └── feed_mode.h     # 数据喂入模式配置
│       ├── data_adapter.h      # 多源数据格式转换
│       ├── databus.h           # 数据总线通信系统
│       ├── log.h               # 分级日志系统
│       ├── math/               # 数学工具库
│       │   ├── distributions.h     # 概率分布计算
│       │   ├── incremental_rank.h  # 增量排名算法
│       │   ├── linear_algebra.h    # 线性代数工具
│       │   ├── numeric_utils.h     # 数值工具
│       │   ├── sliding_normal_eq.h # 增量法正则方程
│       │   └── statistics.h        # 统计计算
│       ├── nms_bucket_aggregator.h # 时间桶聚合器
│       ├── trading_time.h      # 交易时间处理
│       └── types.h             # 统一数据类型定义（QuoteDepth / Transaction / Entrust / CombinedTick / Bar 等）
├── src/                        # 源文件实现
│   ├── basic_factors/          # 基础因子实现
│   │   ├── tick_trans_orders.cpp
│   │   └── tick_trans_orders.h
│   ├── bridge/
│   │   └── ingress.cpp         # 数据入口实现
│   ├── config/                 # 运行时配置
│   │   ├── runtime_config.cpp
│   │   ├── runtime_config.h
│   │   └── runtime_config.ini
│   ├── stat_factors/           # 统计因子实现
│   │   ├── gaussian_copula_factor.cpp
│   │   ├── gaussian_copula_factor.h
│   │   ├── granger_causality_factor.cpp
│   │   └── granger_causality_factor.h
│   └── utils/                  # 工具类实现
│       ├── data_adapter.cpp
│       ├── log.cpp
│       ├── nms_bucket_aggregator.cpp
│       └── trading_time.cpp
├── tests/                      # 测试代码
│   ├── basic_factors_tests/
│   │   └── tick_trans_orders_test.cpp
│   ├── stat_factors_tests/
│   │   ├── gaussian_copula_factor_test.cpp
│   │   └── granger_causality_factor_test.cpp
│   ├── integration/
│   │   └── demo_min_e2e_test.cpp   # Demo 级 E2E 测试
│   ├── utils/
│   │   ├── data_gen.h              # 测试数据生成器
│   │   ├── test_config.cpp
│   │   ├── test_config.h
│   │   └── test_config.ini
│   ├── data/                       # 测试用样例数据
│   │   ├── bars_minute_csv.csv
│   │   ├── snapshot_quotes_csv.csv
│   │   └── transactions_tick_csv.csv
│   ├── factor_compute_test.cpp
│   ├── gtest_printer_zh.h
│   └── test_wait.cpp
└── third_party/               # 第三方依赖（优先使用仓库内版本）
    ├── boost/                 # 通过 bcp 导出的最小 Boost 头（必须存在）
    ├── eigen/                 # Eigen 线性代数库（头文件）
    ├── googletest/            # GoogleTest 测试框架
    └── spdlog/                # spdlog 日志库（可选）
```

---

## 🎪 核心组件深度解析

### 1. 因子接口 (IFactor) - 统一的因子契约

**设计目标**：为所有因子提供统一的接口规范，确保代码的一致性和可维护性。

**核心接口（与 `include/ifactor.h` 设计保持一致）**：
```cpp
// IFactor：所有因子的抽象基类，定义了数据输入与刷新/元数据等接口
class IFactor {
public:
    virtual ~IFactor() = default;

    // —— 核心数据处理接口 ——
    // L2 行情（快照/盘口）
    virtual void on_quote(const QuoteDepth& q) = 0;

    // 统一逐笔入口（成交 / 委托 都通过 CombinedTick 喂入）
    virtual void on_tick(const CombinedTick& x) = 0;

    // 若调用方已有 Transaction / Entrust，可以通过适配函数喂入：
    // （典型实现：内部构造 CombinedTick 后转调 on_tick）
    void on_tick(const Transaction& t); // 适配函数
    void on_tick(const Entrust& e);     // 适配函数

    // Bar / 时间桶 回调（如按分钟 / 日线喂入）
    virtual void on_bar(const Bar& b) {}

    // —— 强制刷新接口 ——
    // 在收盘/日切/策略要求的特殊时刻，强制产出某个 code 当前桶的结果
    virtual bool force_flush(const std::string& code) = 0;

    // —— 元数据接口 ——
    // 因子的人类可读名称（用于日志和监控）
    virtual std::string get_name() const = 0;

    // 这个因子关心的标的集合（可为空，表示“遇到什么算什么”）
    virtual const std::vector<std::string>& get_codes() const = 0;
};
```

**实现特点**：

- **事件驱动设计**：使用 `on_quote` + `on_tick(CombinedTick)` 处理不同类型的数据
- **统一 Tick 入口**：成交 / 委托先在入口层合并为 `CombinedTick`，简化因子实现
- **强制刷新机制**：`force_flush` 确保在收盘或特定时刻输出计算结果
- **多代码支持**：单个因子实例可以同时监控多个 code

**BaseFactor 基类（公共元数据 + code 初始化钩子）**：
```cpp
class BaseFactor : public IFactor {
protected:
    std::vector<std::string> _codes;          // 关心的标的集合
    std::string              _name;           // 因子名称
    std::unordered_set<std::string> _known_codes; // 已初始化过的 code

public:
    BaseFactor(const std::string& name, std::vector<std::string> codes)
        : _codes(std::move(codes)), _name(name) {}

    // 默认返回因子名称
    std::string get_name() const override { return _name; }

    // 默认返回关注的标的集合
    const std::vector<std::string>& get_codes() const override { return _codes; }

protected:
    /**
     * @brief 确保某个 code 的内部状态已初始化（只在首次见到该 code 时触发）
     * 典型用途：
     *  - 为该 code 创建窗口/缓存/统计器
     *  - 注册 DataBus topic 的订阅/发布钩子
     *  - 建立跨模块的索引（如 code->state 映射）
     */
    void ensure_code(const std::string& code) {
        if (_known_codes.find(code) != _known_codes.end()) return;
        _known_codes.insert(code);
        on_code_added(code);
    }

    /**
     * @brief 派生类可覆盖此钩子，完成 code 级别的自定义初始化
     * 缺省实现为空。
     */
    virtual void on_code_added(const std::string& /*code*/) {}
};
```

> **code 是什么？**  
> 表示“标的物唯一标识符”。例如：股票 `600000.SH`、期货 `IF2506`、指数/ETF `000300.SH`、加密交易对 `BTCUSDT` 等。框架按 **code 维度** 隔离状态与计算。


### 2. 数据总线 (DataBus) - 因子通信的神经系统

**设计目标**：实现因子间的松耦合通信，支持类型安全的数据交换和时间对齐。

**核心特性**：
- **类型安全**：每个 Topic 绑定特定数据类型，编译期检查
- **时间对齐**：推荐以“**桶结束时间**”发布和读取
- **环形历史**：可设容量，自动淘汰旧数据
- **多种访问模式**：拉取、订阅、阻塞等待皆可

**关键 API**：
```cpp
// 1) 注册 Topic：声明某个主题（如 "zyd/amount"）会发布 T 类型的数据
template<typename T>
void register_topic(const std::string& topic, size_t capacity=120);

// 2) 发布数据：在 topic/code 上发布一个时间戳 ts_ms 的值 value
template<typename T>
void publish(const std::string& topic, const std::string& code, 
             int64_t ts_ms, const T& value);

// 3) 读取最新数据：拿到某个 topic/code 的最后一条数据
template<typename T>
bool get_latest(const std::string& topic, const std::string& code, 
                T& out, int64_t* ts_ms=nullptr);

// 4) 按精确时间戳读取：常用于“按桶结束时间”对齐后的消费
template<typename T>
bool get_by_time_exact(const std::string& topic, const std::string& code, 
                       int64_t ts_ms, T& out);

// 5) 读取最近 N 条数据：用于回看短历史
template<typename T>
std::vector<std::pair<int64_t, T>> get_last_n(const std::string& topic, 
                                              const std::string& code, size_t n);

// 6) 订阅：当某 topic/code 有新数据发布时回调 cb（推模式）
template<typename T>
void subscribe(const std::string& topic, const std::string& code,
               std::function<void(const std::string&, int64_t, const T&)> cb);

// 7) 等待：阻塞直到到达目标时间戳（精确/不早于）
template<typename T>
bool wait_for_time_exact(const std::string& topic, const std::string& code,
                         int64_t ts_ms, T& out, int64_t timeout_ms = 1000);

template<typename T>
bool wait_for_time_at_least(const std::string& topic, const std::string& code,
                            int64_t ts_ms, T& out, int64_t timeout_ms = 1000);
```

> **建议**：发布与消费统一使用“桶结束时间”作为时间戳，这样不同因子之间可以严格时间对齐。

---

### 3. 时间桶聚合器 (NmsBucketAggregator) - 时序对齐引擎

**设计目标**：将高频数据聚合到固定时间桶，为因子计算提供时间对齐的输入。

**核心接口**：
```cpp
// NmsBucketAggregator：把高频事件聚合到固定毫秒粒度的时间桶中
class NmsBucketAggregator {
public:
    explicit NmsBucketAggregator(int64_t bucket_ms=1000); // 构造时设定桶大小（默认 1s）
    
    // —— 数据输入接口 ——
    // on_quote：处理一条 L2 行情（会用于累计 amount/volume、更新 midprice）
    void on_quote(const QuoteDepth& q);
    // on_transaction：处理一条逐笔成交（可用于切片/辅助逻辑）
    void on_transaction(const Transaction& t);
    // on_entrust：处理一条逐笔委托（可用于切片/辅助逻辑）
    void on_entrust(const Entrust& e);
    
    // —— 桶产出接口 ——
    // flush_if_crossed：如果 now_ms 已经跨过当前桶边界，则产出上一个桶的结果到 out，并返回 true
    bool flush_if_crossed(int64_t now_ms, BucketOutputs& out);
    // force_flush：强制产出当前桶的结果（一般用于收盘/日切）
    bool force_flush(BucketOutputs& out);
    // ensure_bucket：确保 ts_ms 所在的桶已存在（必要时初始化），并返回该桶的聚合结果句柄/快照
    bool ensure_bucket(int64_t ts_ms, BucketOutputs& out);
};
```

**聚合输出结构**：
```cpp
// BucketOutputs：一个“时间桶”的汇总结果
struct BucketOutputs {
    double amount_sum = 0.0;              // 成交额累计（turnover 累计）
    int64_t volume_sum = 0;               // 成交量累计
    double midprice_last = 0.0;           // 桶内最后一个中间价（(bid+ask)/2）
    std::vector<Transaction> trans;       // 桶内逐笔成交
    std::vector<Entrust> orders;          // 桶内逐笔委托
    int64_t bucket_start_ms = 0;          // 桶开始时间（毫秒）
    int64_t bucket_end_ms = 0;            // 桶结束时间（毫秒）——建议用于发布时间戳
};
```

**增量计算优化**：
- 维护上一次行情状态，避免重复计算
- 支持跨交易日的 volume/turnover 重置
- 智能桶边界检测，确保数据完整性

---

### 4. 模块化工具组件

#### 数据类型 (types.h)
```cpp
// —— L2 行情数据 ——
struct QuoteDepth {
    std::string instrument_id{};     // 合约/标的代码（即 code）
    int64_t data_time_ms{0};         // 数据时间戳（毫秒）
    int trading_day{0};              // 交易日（YYYYMMDD）
    uint64_t volume{0};              // 成交量（累计）
    double turnover{0.0};            // 成交额（累计）
    double bid_price{0.0};           // 买一价
    double ask_price{0.0};           // 卖一价
};

// —— 逐笔成交记录 ——
struct Transaction {
    std::string instrument_id{};     // 标的代码
    int64_t data_time_ms{0};         // 时间（毫秒）
    uint64_t main_seq{0};            // 主推序号
    double price{0.0};               // 成交价格
    int side{0};                     // 买卖方向
    uint64_t volume{0};              // 成交数量
    uint64_t bid_no{0};              // 买方订单号
    uint64_t ask_no{0};              // 卖方订单号
};

// —— 逐笔委托记录 ——
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

> 说明：
> - 虽然 IFactor 接口已经不再提供 `on_transaction` / `on_entrust` 这两个虚函数，  
    >   但 `Transaction` / `Entrust` 这两类结构体仍然是框架内部的重要事件模型。
> - 它们主要由 `DataAdapter` 和 `NmsBucketAggregator` 等组件使用：从混合逐笔表中构造事件、在时间桶内保存完整逐笔明细等。
> - 在因子实现层，统一入口仍然是 `on_tick(const CombinedTick&)`，是否需要进一步区分成交/委托取决于具体业务逻辑。

#### 交易时间工具 (trading_time.h/cpp)
```cpp
// TradingTime：提供 A 股常用交易时间判断/推算工具
class TradingTime {
public:
    static bool in_trading_session_ms(int64_t ms);        // 是否在连续竞价时段
    static bool in_call_auction_ms(int64_t ms);           // 是否在集合竞价时段
    static int64_t next_trading_session_start(int64_t current_ms); // 下一个连续竞价开始时间
    static bool is_valid_trading_day(int trading_day);    // 是否为有效交易日（排周末/节假日）
};
```

#### 数学工具 (math)
```cpp
// Statistics：常见统计计算（均值/方差/中位数/分位数/相关性/滑窗均值等）
// 注意：大窗口请考虑 O(1)/O(log n) 的增量更新实现，以避免高额内存与 CPU 占用。
template<typename T>
class Statistics {
public:
    template<typename Container> static double mean(const Container& data);
    template<typename Container> static double stddev(const Container& data);
    template<typename Container> static double median(const Container& data);
    template<typename Container> static double quantile(const Container& data, double percentile);
    template<typename Container1, typename Container2> 
    static double correlation(const Container1& x, const Container2& y);
    template<typename Container> 
    static std::vector<double> rolling_mean(const Container& data, size_t window_size);
};
```
**增量排名计算** (`incremental_rank.h`)：
- O(log n) 时间复杂度的滑动窗口排名计算
- 支持中位秩、分位数等统计量
- 内存高效的排序维护

除此之外，当前 `math` 目录下还包含以下模块（这里按实际头文件简单介绍用途）：

- `numeric_utils.h`：提供收益率计算、近似比较等数值工具，对一些边界情况做了数值稳定性处理。
- `linear_algebra.h`：基于 Eigen 的矩阵/向量运算工具，用于协方差矩阵、特征分解等线性代数计算，在高斯/格兰杰等因子里会用到。
- `sliding_normal_eq.h`：与滑窗普通最小二乘（OLS）相关的辅助工具，用于在滑动窗口上构造和更新回归模型的法方程。
- `distributions.h`：分布函数与 p 值计算工具，例如 F 分布右尾概率等，在统计检验类因子（如格兰杰因子）中用于将统计量转化为显著性水平。

---

### 5. 数据适配器 (DataAdapter) - 格式转换层
```cpp
// DataAdapter：不同交易所/源格式 → 统一的内部结构
class DataAdapter {
public:
    // —— 快照转换 ——
    static QuoteDepth from_snapshot_sh(const SnapshotStockSH& snapshot);    // 上交所
    static QuoteDepth from_snapshot_sz(const std_SnapshotStockSZ& snapshot);// 深交所
    
    // —— 成交转换 ——
    static Transaction from_ord_exec(const OrdAndExeInfo& ord_exec);
    
    // —— 混合逐笔识别与转换 ——（OrdAndExeInfo 是“成交+委托一张表”的逐笔记录）
    static bool is_trade(const OrdAndExeInfo& x);
    static Transaction to_transaction(const OrdAndExeInfo& x);
    static Entrust to_entrust(const OrdAndExeInfo& x);
    
    // —— 价格标准化 ——
    static double normalize_price(uint32_t raw_price);
};
```

---

## 🚀 快速开始

### 环境要求
- **操作系统**：Linux / Windows / macOS
- **编译器**：C++17（GCC 7+ / Clang 5+ / MSVC 2019+）
- **构建工具**：CMake 3.15+（当前 CMakeLists.txt 要求）
- **第三方**：
    - 必需：`third_party/boost`（通过 bcp 导出的最小 Boost 头文件）
    - 推荐：`third_party/eigen`（线性代数）、`third_party/googletest`（若开启测试）、`third_party/spdlog`（更好的日志输出）

### 构建项目
```bash
# 1) 克隆项目
git clone git@gitee.com:tangjian8109/factors_lib.git
cd factors_lib

# 2) 生成构建目录
mkdir build && cd build

# 3) 配置（可按需添加 -D 选项）
cmake .. -DCMAKE_BUILD_TYPE=Release

# 4) 编译
cmake --build . -j

# 5) 运行测试（若构建了测试）
ctest --output-on-failure
```

**Windows (VS 2019+)**
```bat
mkdir build && cd build
cmake .. -G "Visual Studio 16 2019" -A x64
cmake --build . --config Release
```

## 🧩 与 Demo 的集成

**方式 1：Demo 把本库作为子目录（源码集成）**
```cmake
# Demo/CMakeLists.txt
# 说明：将本仓库放入 external/factors_lib，然后在 Demo 中 add_subdirectory
add_subdirectory(external/factors_lib)
add_executable(demo main.cpp)
# 链接到本库导出的目标（示例）
target_link_libraries(demo PRIVATE factor_basic factorlib_utils)
```

**方式 2：同工作区构建（推荐做法）**
- 若 Demo 与本库放在同一工作区，可以在上层 CMake 中把 `../demo_header` 加入 include 路径，
  并链接 `factor_basic` / `factorlib_utils`；本仓 `CMakeLists.txt` 中已留有注释示例。

---

---

## ⚙️ CMake 选项总览

| 选项 | 默认 | 说明 |
|---|---:|---|
| `FACTORLIB_BUILD_TESTS`      | `ON`  | 是否启用测试相关目标的构建总开关 |
| `FACTORLIB_BUILD_UNIT_TESTS` | `ON`  | 是否构建单元测试可执行文件 `run_tests`（仅在 `FACTORLIB_BUILD_TESTS=ON` 时生效） |
| `FACTORLIB_WITH_DEMO_E2E`    | `OFF` | 是否构建 E2E 测试 `run_e2e`（依赖 demo_header；仅用于集成验证） |
| `FACTORLIB_USE_THIRD_PARTY`  | `ON`  | **优先使用** `third_party/` 下的 Eigen / GTest / spdlog 等依赖 |
| `FACTORLIB_ENABLE_TRACE_DEBUG` | `OFF` | 是否**编译进** TRACE/DEBUG 日志（OFF 时会定义 `FACTORLIB_NO_DEBUG_TRACE=1` 以裁掉相关代码） |

> **日志编译开关说明**
> - 当 `FACTORLIB_ENABLE_TRACE_DEBUG=OFF`（默认）时：CMake 定义 `FACTORLIB_NO_DEBUG_TRACE=1`，`LOG_TRACE/LOG_DEBUG` 宏在编译期被裁掉；若存在 spdlog，建议设置 `SPDLOG_ACTIVE_LEVEL=INFO`。
> - 当 `FACTORLIB_ENABLE_TRACE_DEBUG=ON` 时：不定义裁剪宏，可输出 TRACE/DEBUG 以便调试。


---

## 🧭 设计保证

1. **时间对齐**：所有发布数据使用**桶结束时间**作为时间戳；跨桶边界自动 `flush`，保证因子消费一致性。
2. **强制刷新**：交易收盘或特定事件触发 `force_flush(code)`，确保当日末状态落盘。
3. **窗口策略**：滑动窗口在**未满**时的行为可配置（不发布 / 发布 NaN / 发布部分统计）。
4. **跨交易日**：成交量/成交额等在新交易日自动重置，避免日内累计串日。
5. **单线程假设**：当前实现**默认单线程**使用（无锁）。
6. **异常处理**：输入异常（时间戳倒退、负价格）记录 `LOG_WARN/ERROR`，并跳过。

---

## 🔭 可观测性与日志

- 使用统一日志宏：`LOG_TRACE/DEBUG/INFO/WARN/ERROR`（见 `include/utils/log.h`）。
- 默认 **不编译** TRACE/DEBUG：`FACTORLIB_ENABLE_TRACE_DEBUG=OFF` → 对性能/体积零成本。
- 存在 `third_party/spdlog` 时启用彩色控制台输出；否则回退到 `fprintf(stderr, ...)`。
- 建议 Demo 输出指标：发布/订阅 QPS、丢弃计数、窗口滞后数。

---

## 🧱 扩展因子约定

**Checklist 与示例实现（含中文注释）**：
```cpp
// =====================
// 约定 1：命名
//   - 类名以 *Factor 结尾（如 GaussianCopulaFactor）
//   - 文件名与类名一致，便于查找与导航
// 约定 2：接口
//   - 至少实现 IFactor 的 on_quote 和 on_tick(CombinedTick)
//   - 一般不再直接 override on_transaction/on_entrust，
//     如确需区分成交/委托，建议在公共入口或工具函数中根据 CombinedTick::kind 做一次分流，
//     尽量避免在每个因子里重复手写 `if (x.kind == ...)`，新的因子实现可直接基于 CombinedTick 字段编写逻辑
//   - 若需要在收盘/日切产出结果，重写 force_flush
// 约定 3：按 code 初始化
//   - 首次见到某个 code 时，创建该 code 的聚合器/窗口等状态
// 约定 4：Topic 命名
//   - 推荐 "namespace/name"（如 "zyd/amount"），跨模块统一
// 约定 5：时间戳
//   - 统一使用 ms 时间戳（int64_t），并尽量以“桶结束时间”作为对齐时间
// =====================
```

下面给出一个与当前实现风格一致的示例（伪代码，删掉了与业务无关的细节），展示如何基于 `NmsBucketAggregator` 写一个逐笔聚合因子：

```cpp
class TickAmountFactor : public BaseFactor {
public:
    struct PerCodeState {
        NmsBucketAggregator agg;   // 时间桶聚合器
        // 这里可以再扩展自己的缓存/中间状态
    };

    explicit TickAmountFactor(std::vector<std::string> codes)
        : BaseFactor("TickAmountFactor", std::move(codes)) {}

    // —— L2 行情数据 ——（可选）
    void on_quote(const QuoteDepth& q) override {
        ensure_code(q.instrument_id);
        auto& s = _state[q.instrument_id];

        // 1) 先尝试“跨桶产出”：若当前时间已跨过上一个桶，先把上一个桶产出发布
        BucketOutputs out;
        if (s.agg.flush_if_crossed(q.data_time_ms, out)) {
            publish_results(q.instrument_id, out); // 发布结果（见下方函数）
        }

        // 2) 再把本条行情纳入聚合器
        s.agg.on_quote(q);
    }

    // —— 统一 Tick 入口：收到一条 CombinedTick（成交 or 委托） ——
    void on_tick(const CombinedTick& x) override {
        ensure_code(x.instrument_id);
        auto& s = _state[x.instrument_id];

        // 1) 与 on_quote 一样，先按时间检测是否需要产出上一个桶
        BucketOutputs out;
        if (s.agg.flush_if_crossed(x.data_time_ms, out)) {
            publish_results(x.instrument_id, out);
        }

        // 2) 这里的示例只演示基于 CombinedTick 的增量逻辑：
        //    可以直接使用 x.price / x.volume / x.data_time_ms 等字段构建自己的状态，
        //    而不必在每个因子里都手写 `if (x.kind == ...)` 分支。
        //    实际工程中，可以参考当前 Tick/高斯/格兰杰因子，在公共入口层统一做一次 kind 分流，
        //    然后再按需调用本因子的 on_tick(CombinedTick)。
    }


    // —— Bar 回调：如按分钟 Bar 进行补充对齐/收口，可按需实现 ——
    void on_bar(const Bar& b) override {
        // 可选：根据 Bar 触发额外逻辑
    }

    // —— 强制刷新：用于收盘/日切等时刻 —— 
    bool force_flush(const std::string& code) override {
        auto it = _state.find(code);
        if (it == _state.end()) return false;

        BucketOutputs out;
        if (!it->second.agg.force_flush(out)) {
            return false;
        }
        publish_results(code, out);
        return true;
    }

private:
    std::unordered_map<std::string, PerCodeState> _state;

    void publish_results(const std::string& code, const BucketOutputs& out) {
        // 在这里把聚合结果写入 DataBus / 日志 / 下游系统
        // 例如：
        //   _bus.publish("zyd/amount", code, out.amount);
    }
};
```

> **建议**：如果要写新的因子，一般可以：
> 1. 直接从现有的 `TickTransOrders` / `GaussianCopulaFactor` / `GrangerCausalityFactor`
     >    中拷一份骨架；
> 2. 替换掉聚合/统计部分逻辑；
> 3. 保留 `ensure_code` + `force_flush` + DataBus 发布等约定写法。


## ❓ FAQ

**Q: 为什么拿不到最新值？**  
A: 请确认消费端读取的是**桶结束时间**对应的数据；若窗口未满且策略为“不发布”，会在窗口满足后才产出。

**Q: 窗口未满如何处理？**  
A: 支持三种模式：不发布 / 发布 NaN / 发布部分统计。默认建议“不发布”，避免误用。

**Q: 日志太多怎么办？**  
A: 默认已编译期移除了 TRACE/DEBUG。若仍多，可在运行时将级别设为 `WARN` 或 `ERROR`，并保持 `FACTORLIB_ENABLE_TRACE_DEBUG=OFF`。

---

## 🚀 基本使用示例

> 说明：示例延续你当前的 include 路径风格；日志统一用 `LOG_*` 宏。

```cpp
#include "factors_lib/include/ifactor.h"          // 因子接口/基类
#include "factors_lib/include/utils/databus.h"    // 数据总线
#include "factors_lib/include/utils/log.h"        // 日志宏

int main() {
    // —— 1) 打一条 INFO 级日志（默认不会编译 TRACE/DEBUG） ——
    LOG_INFO("开始因子计算");

    // —— 2) 注册数据总线主题（只需一次） ——
    factors_lib::TickTransOrders::register_topics(120);   // Tick → 聚合产出
    factors_lib::GaussianCopulaFactor::register_topics(60);// 高斯 Copula 产出

    // —— 3) 配置并创建一个基础因子 ——
    factors_lib::TickTransOrdersConfig tick_cfg;
    tick_cfg.bucket_size_ms = 1000;        // 1 秒时间桶
    tick_cfg.emit_tick_interval = true;    // 是否按 tick 间隔输出

    std::vector<std::string> codes = {"000001.SZ", "600000.SH"};
    auto tick_factor = factors_lib::TickTransOrders(tick_cfg, codes);

    // —— 4) 构造并处理一条行情 ——
    factors_lib::QuoteDepth quote;
    quote.instrument_id = "000001.SZ";     // code
    quote.data_time_ms  = 1704065400000;   // 2024-01-01 09:30:00
    quote.bid_price     = 10.0;
    quote.ask_price     = 10.2;
    quote.volume        = 1000;
    quote.turnover      = 10000.0;

    tick_factor.on_quote(quote);           // 投喂数据

    // —— 5) 从 DataBus 读取最新结果 ——
    auto& bus = factors_lib::DataBus::instance();
    double amount = 0.0; int64_t ts = 0;
    if (bus.get_latest<double>("zyd/amount", "000001.SZ", amount, &ts)) {
        LOG_INFO("股票 {} 在 {} 的成交额: {}", "000001.SZ", ts, amount);
    }

    // —— 6) 订阅主题更新（推模式） ——
    bus.subscribe<double>("zyd/amount", "000001.SZ", 
        [](const std::string& code, int64_t ts2, const double& value) {
            std::cout << "[订阅] 实时成交额更新: " 
                      << code << " @ " << ts2 << " = " << value << std::endl;
        });

    return 0;
}
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
    void on_tick(const CombinedTick& x) override;
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

void MyCustomFactor::on_tick(const CombinedTick& x) {
    ensure_code(x.instrument_id);
    // 这里仅给出基于 CombinedTick 的示意写法：
    //  - 可以按需要使用 x.price / x.volume / x.data_time_ms 等字段
    //  - 如果在公共入口层已经按 kind 拆分并维护了聚合状态，则这里可以专注于“因子本身”的逻辑
    (void)x;
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

---

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

```cpp
// 示例：使用“桶结束时间”进行严格时间对齐
void process_data_with_time_alignment() {
    // 计算当前时间所在桶的结束时间（具体实现依赖你的时间工具）
    int64_t current_bucket_end = get_current_bucket_end();

    // 1) 基础因子计算（产生基础特征，发布到 DataBus）
    base_factor.on_quote(quote);

    // 2) 复杂因子等待基础特征在 current_bucket_end 产出后再消费
    double base_output = 0.0;
    if (bus.wait_for_time_exact<double>("base/topic", "000001.SZ",
                                        current_bucket_end, base_output, /*timeout_ms=*/1000)) {
        // 3) 时间对齐成功，进行组合特征计算
        complex_factor.compute(base_output);
    } else {
        // 超时：说明基础特征未在该桶结束时产出，可选择降级或跳过
        LOG_WARN("等待 base/topic@{} 超时，跳过本桶", current_bucket_end);
    }
}
```

**时间对齐示意：**
```
时间轴: 09:30:00.000 ──── 09:30:01.000 ──── 09:30:02.000 ────→
桶划分:    桶1      │       桶2       │       桶3       │
           ↓       ↓        ↓        ↓        ↓        ↓
因子A产出: 值A1 @ 09:30:01.000 │ 值A2 @ 09:30:02.000 │ ...
因子B产出: 值B1 @ 09:30:01.000 │ 值B2 @ 09:30:02.000 │ ...
```

---

## 🧪 测试策略

### 单元测试框架
项目使用 GoogleTest 框架，提供中文输出的测试结果。

**测试代码结构（片段）**：
```cpp
#include <gtest/gtest.h>
#include "utils/data_gen.h"

// 使用 Test Fixture 管理因子与测试数据
class TickTransOrdersTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 1) 注册 Topic（只需一次）
        factors_lib::TickTransOrders::register_topics(50);
        // 2) 创建被测因子
        _factor = std::make_unique<factors_lib::TickTransOrders>(_cfg, std::vector<std::string>{"TEST001"});
    }
    
    // 测试用配置与被测对象
    factors_lib::TickTransOrdersConfig _cfg;
    std::unique_ptr<factors_lib::TickTransOrders> _factor;
};

TEST_F(TickTransOrdersTest, BasicAggregation) {
    // 1) 生成一段可预测的数据序列（quotes/trans/orders）
    auto series = factors_lib::testutil::make_series_basic("TEST001", 
                                                          /*start_ms=*/1704065400000,
                                                          /*bucket_ms=*/1000);
    // 2) 投喂行情
    for (const auto& quote : series.quotes) {
        _factor->on_quote(quote);
    }
    // 3) 验证聚合产出（以成交额为例）
    double amount = 0.0;
    ASSERT_TRUE(factors_lib::DataBus::instance()
                .get_latest<double>("zyd/amount", "TEST001", amount));
    EXPECT_NEAR(amount, /*期望值*/ 200000.0, 1e-6);
}
```

### 测试数据生成工具（`data_gen.h`）
```cpp
namespace factors_lib::testutil {

// 把时分秒毫秒转成当天毫秒时间戳（简化示例）
inline int64_t hms_ms(int H, int M, int S, int ms = 0) {
    return ((H * 3600LL + M * 60LL + S) * 1000LL + ms);
}

// 用于单元测试的序列结构
struct Series {
  std::vector<QuoteDepth>   quotes;  // 模拟的 L2 行情
  std::vector<Transaction>  trans;   // 模拟的 逐笔成交
  std::vector<Entrust>      orders;  // 模拟的 逐笔委托
};

// 生成一套“可预测”的序列，便于验证聚合后的结果
Series make_series_basic(const std::string& code, int64_t start_ms, int64_t bucket_ms) {
    Series s;
    // TODO：构造若干条 quote/trans/orders，确保每个时间桶的累计值是已知的
    // 例如：每桶固定 2 条 quote，每条 turnover/volume 固定增量
    return s;
}

} // namespace factors_lib::testutil
```

---

## 📈 实际案例：高斯 Copula 因子

### 算法原理（要点）
1. **正态分数**：将 OFI、成交量、收益率转换为正态分数（rank → N(0,1) 分位）。
2. **协方差估计**：增量估计多变量高斯分布的协方差矩阵。
3. **条件期望**：给定 (OFI, 成交量) 预测收益率的条件期望。
4. **逆变换**：把正态空间结果变回原始收益率尺度。

**数学公式**：
```
Z_return | Z_ofi, Z_volume ~ N(μ_cond, Σ_cond)
其中：
μ_cond = μ_return + Σ_{return,[ofi,volume]} · Σ_{[ofi,volume]}^{-1} · (Z_obs - μ_obs)
```

### 增量实现片段
```cpp
// IncrementalState：示例性增量计算（伪代码）
void IncrementalState::update_data(double ofi, double volume, double ret) {
    // 1) 秩统计：把原始量（ofi/volume/ret）送入“滑动秩”计算器（窗口大小 = window_size）
    ofi_rank_calc.push(ofi, window_size);
    volume_rank_calc.push(volume, window_size);
    return_rank_calc.push(ret, window_size);
    
    // 2) 当窗口“已满”时，计算对应的正态分数（中位秩 → 正态分位）
    if (is_window_full()) {
        double z_ofi = math::Distributions::normal_quantile(ofi_rank_calc.median_rank(ofi));
        double z_volume = math::Distributions::normal_quantile(volume_rank_calc.median_rank(volume));
        double z_return = math::Distributions::normal_quantile(return_rank_calc.median_rank(ret));
            
        Eigen::Vector3d normal_score(z_ofi, z_volume, z_return);
        // 3) 增量更新协方差估计器（避免保留完整窗口历史）
        cov_calc.push(normal_score);
    }
}
```

**优势**：O(log n) 更新、低内存、数值稳定（可对协方差做正则化）。

---

## ⚡ 最佳实践和性能优化

### 命名规范
```
{作者或团队}/{因子类型}/{具体指标}
示例：
zyd/amount                  # 成交额
zyd/tick/trans             # Tick成交切片  
gaussian_copula/prediction # Copula 预测值
```

**代码风格**：类名 `PascalCase`（如 `TickTransOrders`）；变量 `snake_case`（如 `bucket_size_ms`）；常量 `UPPER_SNAKE_CASE`。

### 错误处理
```cpp
// ensure_code：确保首次见到某个 code 时已建好状态
void ensure_code(const std::string& code) {
    if (_aggregators.find(code) == _aggregators.end()) {
        _aggregators.emplace(code, NmsBucketAggregator(_cfg.bucket_size_ms));
        LOG_DEBUG("初始化代码 {} 的聚合器", code);
    }
}

// safe_publish：包装发布逻辑，统一处理异常
template <typename T>
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

### 性能考虑（示例）
```cpp
// 1) 引用遍历，避免不必要拷贝
void process_large_data(const std::vector<QuoteDepth>& quotes) {
    for (const auto& quote : quotes) {
        factor.on_quote(quote); // const 引用访问
    }
}

// 2) 热路径避免动态分配：预分配/复用对象
void on_quote(const QuoteDepth& q) override {
    thread_local BucketOutputs out;  // 线程局部存放，避免反复分配
    if (_aggregator.flush_if_crossed(q.data_time_ms, out)) {
        publish_results(out);
        out = BucketOutputs{};       // 原地重置，而非重新分配
    }
}
```

### 内存管理（智能指针）
```cpp
class FactorManager {
private:
    std::vector<std::unique_ptr<IFactor>> _factors; // 所有因子由容器统一持有
    
public:
    void add_factor(std::unique_ptr<IFactor> factor) {
        _factors.push_back(std::move(factor));      // 所有权移动，自动析构
    }
};
```

---

## 🔍 故障排除和调试

### 常见问题
```
问题：尝试访问未注册的主题 'unknown/topic'
解决：在因子构造或入口处调用 register_topics()
```
```
问题：复杂因子读不到基础因子输出
原因：时间戳未对齐
解决：统一使用“桶结束时间”作为发布与消费时间戳
```
```
问题：怀疑内存泄漏
解决：使用 Valgrind / AddressSanitizer；控制 DataBus capacity；尽量采用增量算法
```

### 调试技巧
```cpp
// 添加 TRACE 跟踪（默认不会编译进二进制，打开 CMake 开关即可）
LOG_TRACE("开始处理代码 {} 的行情数据", code);
factor.on_quote(quote);
LOG_TRACE("完成处理，当前时间桶: {}", current_bucket_end);
```

```cpp
// 基础数据校验：在热路径之外做，避免影响性能
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

---

## 🔌 扩展指南

### 添加新的数据源（示例）
```cpp
class MyExchangeDataAdapter {
public:
    static QuoteDepth from_my_exchange_format(const MyExchangeSnapshot& snapshot) {
        QuoteDepth q;
        q.instrument_id = snapshot.symbol;     // 统一到 instrument_id
        q.data_time_ms  = snapshot.timestamp;  // 保持毫秒精度
        q.bid_price     = snapshot.best_bid;
        q.ask_price     = snapshot.best_ask;
        q.volume        = snapshot.volume;
        q.turnover      = snapshot.turnover;
        return q;
    }
};
```

### 自定义聚合逻辑（示例）
```cpp
class MyCustomAggregator : public NmsBucketAggregator {
public:
    using NmsBucketAggregator::NmsBucketAggregator;  // 复用父类构造
    
    // 提醒：如果父类方法是 virtual 才能 override，这里仅演示扩展思路
    void on_quote(const QuoteDepth& q) /*override if virtual*/ {
        // 1) 先调用基类行为，确保基础汇总不被破坏
        NmsBucketAggregator::on_quote(q);
        // 2) 在此叠加自定义统计逻辑
        _custom_metric += calculate_custom_value(q);
    }
    
private:
    double _custom_metric = 0.0; // 自定义指标
};
```

---

## 🤝 支持和贡献

### 获取帮助
- **问题报告**：在 Issues 中描述遇到的问题
- **功能请求**：通过 Issue 模板提交新功能建议
- **技术讨论**：参与项目 Discussions 板块

### 贡献指南
1. Fork 项目仓库
2. 创建功能分支：`git checkout -b feature/amazing-feature`
3. 提交更改：`git commit -m 'Add amazing feature'`
4. 推送到分支：`git push origin feature/amazing-feature`
5. 创建 Pull Request

**代码质量要求**：通过所有现有测试；为新增功能补充单元测试；更新相关文档；遵循项目代码风格。

---

## 📚 A. 基础术语：CI / License / Coverage 是什么？

- **CI（Continuous Integration，持续集成）**：每次提交或合并时自动编译 & 运行测试，确保仓库一直“可构建、测试通过”。（Gitee Pipelines 可配置。）
- **License（开源许可）**：约定他人能否/如何使用你的代码。常见如 MIT / Apache-2.0。未声明许可时默认“保留所有权利”，他人法律上不可随意使用。
- **Coverage（测试覆盖率）**：单测运行时统计被执行到的代码比例（如 80% 语句覆盖），帮助发现未被测试的区域。

> 小结：CI = 自动化质量门；License = 合法使用边界；Coverage = 测试有效性量化。

---

## 🏆 架构优势总结

1. **高性能**：增量计算和内存优化设计
2. **可扩展**：模块化架构支持快速添加新因子
3. **类型安全**：编译期检查确保数据一致性
4. **时间对齐**：精确的时间桶聚合机制
5. **松耦合**：数据总线实现因子间解耦
6. **生产就绪**：明确设计保证与分级日志系统

---

**开始使用 FactorLib**：
```bash
git clone <repository-url>
cd factors_lib
mkdir build && cd build
cmake .. -DFACTORLIB_ENABLE_TRACE_DEBUG=OFF
cmake --build . -j
ctest --output-on-failure
```
