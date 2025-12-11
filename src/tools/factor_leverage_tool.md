# 因子杠杆转换工具说明

本文档介绍 `factor_leverage_tool` 的整体流程、配置方式、命令行使用方法与典型输出，方便在后续接入更多因子或调整参数时参考。

## 1. 功能概览

`factor_leverage_tool` 是一个离线分析工具，用于：

1. **回放 CSV K 线数据**：从 `tests/data`（默认）读取各指数/标的的历史 K 线，按照时间顺序送入 `bridge::ingest_kline`。
2. **驱动已实现的因子**：以 `LowFreqReturnFactor` 为例，通过 DataBus 注册并订阅 factor topic，收集原始因子值。
3. **滑窗高斯化 + 杠杆发布**：`FactorLeverageTransformer` 对因子做滑窗 Z 化发布杠杆值，并通过新的 topic 输出。
4. **采样/统计**：
   - 保存原始因子值、杠杆值及其分布图；
   - 记录因子值与次日收益的关系（Scatter 图 + CSV）；
   - 根据回测样本对收益率做“风险对齐 + 阈值网格搜索”，筛出最小满足条件的阈值。
5. **落盘输出**：所有 CSV/PNG 写入 `output/<因子名>/...` 目录，支持多因子并行分析。

## 2. 目录结构

```
src/tools/
├── factor_leverage_transformer.cpp    # 主工具实现
├── factor_tool_config.h/.cpp          # 全局可调参数定义
├── kline_csv_loader.cpp/.h            # CSV 解析
├── factor_leverage_transformer_lib.*  # 滑窗杠杆发布逻辑
└── factor_leverage_tool.md            # 本文档
```

关键输出位于 `output/<factor_name>/`，当 `FACTORLIB_ENABLE_FACTOR_LEVERAGE_PLOTS=ON` 时，会对每张表生成：

- `factor_table_<table>.csv`：单表综合数据，包含 `factor_value / return_value / leverage_value / status`（`status=suppressed` 表示 |z|≤θ）；
- `factor_distribution_<table>.png`：读取同一 CSV 中的 `factor_value` 列；
- `leverage_distribution_<table>.png`：使用 `leverage_value` 列绘制，灰色区域代表被阈值抑制的杠杆，并在图下方标注各 code 的阈值；
- `factor_return_relation_<table>.png`：散点图，直接复用 CSV 中的 `factor_value` 与 `return_value`；
- 以及全局 `leverage_scales.csv`、`leverage_thresholds.csv` 汇总杠杆及阈值统计。

## 3. 配置方式

所有默认参数集中在 `FactorToolConfig`（`src/tools/factor_tool_config.h`）中，可通过修改此文件或设置环境变量覆盖：

| 字段                         | 含义                                               | 默认值 / 覆盖方式                           |
|------------------------------|----------------------------------------------------|---------------------------------------------|
| `factor_name`                | 输出子目录名称                                     | `low_freq_return`，可用 `--factor-name`     |
| `default_leverage_window`    | 滑窗正态化窗口（传给 `SlidingGaussianLeverage`）   | `SlidingGaussianLeverage::kDefaultWindow`   |
| `default_topic_capacity`     | DataBus topic 容量                                 | 64,000                                      |
| `default_lookback_days`      | 裁剪回测样本的天数/条数                            | 0（全量）；可由 env `FACTOR_TOOL_LOOKBACK_DAYS` 或 CLI `--lookback-days` 覆盖 |
| `theta_{min,max,step}`       | 阈值搜索在 Z-score 空间的范围与步长                | 0/2/0.1                                     |
| `max_leverage`               | 杠杆上限（风险对齐后仍不能超过此倍数）             | 2                                           |
| `min_trade_days`             | 阈值生效所需的最少交易样本数                       | 50                                          |

> **提示**：`FACTOR_TOOL_LOOKBACK_DAYS` 只需在运行命令前设定，例如 `set FACTOR_TOOL_LOOKBACK_DAYS=1000`。

## 4. 命令行参数

```
factor_leverage_tool [--data-dir DIR] [--output-dir DIR]
                     [--factor-name NAME]
                     [--codes CODE1,CODE2,...]
                     [--start-date YYYY-MM-DD] [--end-date YYYY-MM-DD]
                     [--window N] [--topic-capacity N]
                     [--lookback-days N]
```

常用选项：

- `--factor-name`：多因子场景下区分输出目录，例如 `--factor-name low_freq_return`。
- `--codes`：指定需要回放的 code 列表（默认取 `tests/data` 中全部）。
- `--start-date/--end-date`：按日过滤 CSV 数据（闭区间，end-date 会加上 23:59:59）。
- `--window`：覆盖滑窗大小，与 `SlidingGaussianLeverage` 保持一致。
- `--lookback-days`：仅保留每个 code 最近 N 条 Bar；若不指定则用配置默认值。

## 5. 阈值搜索与基准对比

在 `search_threshold_for_code`（`src/tools/factor_leverage_transformer.cpp`）中：

1. **标准化**：对某个 code 的所有因子值计算均值/标准差，将因子转成 z-score；
2. **基准收益**：根据 `ret_map` 计算该 code 的“1 倍满仓”平均日收益 `mean_bench`；
3. **阈值遍历**：在 `[theta_min, theta_max]` 范围按 `theta_step` 扫描，流程如下：
   - 过滤掉 |z| ≤ θ 的弱信号，统计开仓天数；
   - 依风险对齐公式计算缩放系数 `c`（满足等方差，并限制最大杠杆 `max_leverage`）；
   - 生成策略收益 `L_t * r_{t+1}`，计算 `mean_R`/`std_R`/`sharpe_annual`；
   - **只要 `n_trades ≥ min_trade_days` 且 `mean_R > mean_bench` 即视为有效**，在所有有效 θ 中选取最小值。

输出文件 `leverage_thresholds.csv` 包含：

```
code,theta_z,theta_raw,c,n_trades,mean_return,std_return,sharpe_annual,
mean_factor,std_factor,baseline_mean,baseline_std,baseline_sharpe,baseline_samples
```

这些字段可直接用于调参或线上阈值配置。

## 6. 常见工作流

1. **准备数据**：将 K 线 CSV 放在 `tests/data/<table>/<code>_....csv`。
2. **运行工具**：
   ```
   cmake --build build --config Release --target factor_leverage_tool -j 12
   build/Release/factor_leverage_tool.exe --codes 000001.SH --factor-name low_freq_return --lookback-days 1000
   ```
3. **查看输出**：在 `output/low_freq_return/` 中查看各类 CSV/PNG；阈值结论在 `leverage_thresholds.csv`。
4. **调参**：根据 `leverage_thresholds.csv` 中的 `theta_z`、`theta_raw`、`mean_return` 等指标，调整 `FactorToolConfig` 或上游因子逻辑。

## 7. 扩展建议

- **接入更多因子**：只需在 CLI 中指定新的 `--factor-name` 并加载对应 factor，实现层保持通用；输出目录会自动区分。
- **可视化字体**：当前 matplotlib 使用默认字体，若需要支持中文标题，可在 `run_plot_script`/`run_relation_plot_script` 中添加 `plt.rcParams['font.sans-serif']` 配置。
- **批量回测**：可以编写脚本循环调用 `factor_leverage_tool`，每次指定不同代码或因子名，统一收集输出。

如需进一步扩展（例如引入多因子组合、更多基准对比方式），建议在此文档基础上记录新增的配置项与流程，保持工具易用性。
