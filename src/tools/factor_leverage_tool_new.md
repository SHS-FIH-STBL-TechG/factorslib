# factor_leverage_tool_new 说明

`factor_leverage_tool_new` 是因子杠杆离线分析工具的新版实现，位于 `src/tools/`。  
目标：在默认 `tests/data` K 线 CSV 上对多个 Kline 因子做 rank→normal 高斯化、阈值搜索、风险对齐，并输出 CSV + 分布图。

## 1. 目录结构

```
src/tools/
├── factor_leverage_tool_new.cpp        # CLI 主程序：读数据、驱动因子、调用优化器、落盘/画图
├── factor_leverage_optimizer.h/.cpp    # 阈值搜索 + 杠杆序列构造核心
├── kline_csv_loader.h/.cpp             # CSV 读取（从旧 src/tools 迁移）
├── factor_leverage_tool_config.h/.cpp  # 因子绑定/默认目录/注册 topics（后续扩因子只改这里）
└── factor_leverage_tool_new.md         # 本文档
```

## 2. 默认行为

直接运行 `factor_leverage_tool_new.exe` 不带参数时：

- `--data_dir` 默认 `tests/data`；
- 自动扫描该目录下全部 `.csv`，按文件名提取 code；
- 遍历 `factor_leverage_tool_config.cpp` 中列出的所有因子；
- 输出根目录默认 `output/factor_leverage/`，每个因子一个子目录；
- 每个 code 输出：
  - `leverage_<code>_<factor>.csv`
  - `distribution_<code>_<factor>.png`（三联图：原始因子、b_raw、最终 leverage）
  - `best_thresholds.csv`（汇总 per-code 最优阈值与统计）

## 3. 命令行参数

```
factor_leverage_tool_new
    [--data_dir DIR]
    [--parquet_dir DIR_OR_FILE]          # Linux only: direct read parquet via pyarrow (no CSV)
    [--factor name1,name2|all]
    [--codes CODE1,CODE2,...]
    [--start YYYY-MM-DD] [--end YYYY-MM-DD]
    [--out_dir DIR]
    [--theta_min 0] [--theta_max 2.5] [--theta_step 0.05]
    [--D 250] [--max_leverage 2]
```

说明：

- `--factor` 为空或 `all` 表示跑全部内置因子；也可传多个逗号分隔的名字。
- `--codes` 为空表示使用 data_dir 下全部 code。
- `--D` 用于风险对齐分母，默认使用原始样本天数（CSV 天数）；手动指定则强制覆盖。
- `--parquet_dir` 默认直读 parquet（不落盘 CSV），支持两类 schema：
  - 列名为 `icode/tradeDate/pxclose...`
  - 列名为数字字符串：`13`=Wind代码, `9`=交易日期, `3000002/3/4/5`=OHLC, `3000006`=涨跌, `3000007`=涨跌幅, `3000008`=成交量, `3000009`=成交额

## 4. 阈值/杠杆逻辑

1. 对 `factor_raw` 做中位数去中心化；
2. 全样本 rank→normal 得到 z 序列；
3. 在 z 空间网格搜索 `theta`，最大化  
   `score = (E_T - 1) / dd_rms`，
   并且要求年均交易日 ≥ 50（按 252 交易日/年折算）；
4. 依据最佳 `theta` 生成原始信号 `b_raw ∈ {0, ±[1,2]}`；
5. 依据风险对齐和最大杠杆上限得到最终杠杆 `leverage = clamp(c * b_raw)`；
6. 通过 `p_high=Φ(theta)` 的经验分位数反推出原值阈值 `x_high/x_low` 并输出。

## 5. 如何扩展/改配置

所有因子绑定、topic 名、默认输出/数据目录均集中在：

`src/tools/factor_leverage_tool_config.cpp`

新增因子时：

1. include 对应因子头；
2. 在 `GetFactorBindings()` 里追加一个 `FactorBinding{key, input_topic, register_topics, create}`；
3. 重新编译即可。

## 6. 依赖

- C++ 部分仅依赖仓库内部 factorlib 与 Boost 头。
- 分布图绘制依赖本机 `python` 与 `matplotlib`：
  ```
  python -m pip install matplotlib
  ```
