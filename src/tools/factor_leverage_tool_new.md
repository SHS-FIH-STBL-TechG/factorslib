# factor_leverage_tool_new 说明

`factor_leverage_tool_new` 是因子杠杆离线分析工具的新版实现，位于 `src/tools/`。  
目标：在默认 `tests/data` K 线 CSV 上对多个 Kline 因子做 rank→normal 高斯化、阈值搜索、风险对齐，并输出 CSV + 分布图。

## 1. 目录结构

```
src/tools/
├── factor_leverage_tool.cpp            # CLI 主程序：读数据、驱动因子、调用优化器、落盘/画图
├── factor_leverage_optimizer.h/.cpp    # 阈值搜索 + 杠杆序列构造核心
├── kline_csv_loader.h/.cpp             # CSV 读取（从旧 src/tools 迁移）
├── factor_leverage_tool_config.h/.cpp  # 因子绑定/默认目录/注册 topics（后续扩因子只改这里）
└── factor_leverage_tool_new.md         # 本文档
```

## 2. 默认行为

直接运行 `factor_leverage_tool_new`（Windows 为 `factor_leverage_tool_new.exe`）不带参数时：

- Windows：`--data_dir` 默认 `tests/data`，自动扫描该目录下全部 `.csv`，按文件名提取 code；
- Linux：若未显式传 `--data_dir`，且存在默认 parquet 目录（代码里探测 `maybe_default_linux_parquet_dir()`），则优先直读 parquet；否则行为同 Windows（读 `--data_dir` 下 CSV）。
- 遍历 `factor_leverage_tool_config.cpp` 中列出的所有因子；
- 输出根目录默认 `output/factor_leverage/`，每个因子一个子目录；
- 每个 code 输出：
  - `leverage_<code>_<factor>.csv`
  - `distribution_<code>_<factor>.png`（仅 Windows 绘制；三联图：原始因子、b_raw、最终 leverage）
  - `best_thresholds.csv`（汇总 per-code 最优阈值与统计）

## 3. 命令行参数

```
factor_leverage_tool_new
    [--data_dir DIR]
    [--parquet_dir DIR_OR_FILE]          # Linux only: direct read parquet via pyarrow (no CSV); when set, overrides --data_dir
    [--factor name1,name2|all]
    [--codes CODE1,CODE2,...]
    [--start YYYY-MM-DD] [--end YYYY-MM-DD]
    [--out_dir DIR]
    [--theta_min 0] [--theta_max 2.5] [--theta_step 0.05]
    [--D 250] [--max_leverage 2]
    [--train 252] [--val 63] [--train_pct 0.75] [--val_pct 0.10] [--step 1] [--zwin 250]
```

说明：

- `--factor` 为空或 `all` 表示跑全部内置因子；也可传多个逗号分隔的名字。
- `--codes` 为空表示使用 data_dir 下全部 code。
- `--D` 用于样本裁剪：若指定则仅使用最近 `D` 天数据；未指定则使用全样本。
- `--train` 为 walk-forward 的滚动窗口长度（每次重新拟合 policy 使用最近 `train` 天的数据）。
- `--val` 为训练窗口内部的验证集长度（用于选阈值）；训练集长度为 `train-val`。
- `--train_pct/--val_pct` 与 `--train/--val` 二选一，且必须成对提供；按每个 code 的总样本数 `N` 自动换算：`train_window=floor((train_pct+val_pct)*N)`、`val_window=floor(val_pct*N)`，剩余为测试集（OOS）。
- `--step` 控制每隔多少天更新一次 policy（其余日期沿用上一期的最优阈值/方向/模式）。
- `--zwin` 为 rank→normal 的滑窗长度（在线高斯化窗口）。
- `--parquet_dir` 默认直读 parquet（不落盘 CSV），支持两类 schema：
  - 列名为 `icode/tradeDate/pxclose...`
  - 列名为数字字符串：`13`=Wind代码, `9`=交易日期, `3000002/3/4/5`=OHLC, `3000006`=涨跌, `3000007`=涨跌幅, `3000008`=成交量, `3000009`=成交额

## 4. 阈值/杠杆逻辑

1. 对 `factor_raw` 做中位数去中心化；
2. 全样本 rank→normal 得到 z 序列；
3. 在 $z$ 空间网格搜索阈值 $\theta$，最大化  
   $$
   Final_{\text{simple}}
   =\left(\sqrt{\frac{252}{\sum_{t=1}^{T}L_t^{2}}}\right)\cdot
   \frac{\frac{252}{T}\sum_{t=1}^{T}L_t r_t}
   {\sqrt{\frac{1}{T}\sum_{t=1}^{T}DD_t^{2}}}
   $$
   并且要求年均交易日 ≥ 50（按 252 交易日/年折算）；
4. 依据最佳 $\theta$ 生成原始信号 $b_{\mathrm{raw}} \in \{0, \pm[1,2]\}$；
5. 本工具当前口径下 $L_t$ 直接取 $b_{\mathrm{raw},t}$（并做 $[-L_{\max},+L_{\max}]$ 裁剪）；
6. 通过 $p_{\mathrm{high}}=\Phi(\theta)$ 的经验分位数反推出原值阈值 $x_{\mathrm{high}}/x_{\mathrm{low}}$ 并输出。

### 4.0 训练集 / 验证集 / 测试集（walk-forward）

本工具在 walk-forward 模式下按“时间顺序”进行三段切分，避免使用未来信息：

- **训练集（train）**：用于分析因子分布、估计 $z_{\mathrm{cap}}$、确定候选交易模式，并在训练集内部计算原值阈值 $x_{\mathrm{low}}/x_{\mathrm{high}}$。
- **验证集（val）**：用于在候选模式下网格搜索 $\theta$，以验证集上的 $Final_{\text{simple}}$ 选择当期最优 policy（$\theta$/mode/polarity 等）。
- **测试集（test / OOS）**：即 walk-forward 输出与评分所用的 OOS 段（从 `t=train` 开始到序列结束）。在该段上按日生成 $L_t$（即 `b_raw/leverage`）并计算最终 `oos_score`。

对应关系：

- 每次更新 policy 时，取最近 `--train` 天作为训练窗口，并切成：
  - 训练集长度：`train_size = --train - --val`
  - 验证集长度：`val_size = --val`
- 评分口径：
  - `avg_val_score`：多次 policy 更新得到的“验证集最优分数”的均值（用于诊断阈值选择质量）
  - `oos_score`：测试集（OOS）上的最终得分

### 4.1 因子评估值（`score`/`oos_score`）最终公式

工具里“因子评估值”对应 `score`（阈值搜索内）/`oos_score`（walk-forward OOS 评估），计算口径一致。设日收益为 $r_t=\mathrm{next\_return}_t$，最终杠杆为 $L_t$（当前口径下 $L_t=b_{\mathrm{raw},t}$，即输出 CSV 中的 `b_raw/leverage`），则最终公式为：

$$
Final_{\text{simple}}
=\left(\sqrt{\frac{252}{\sum_{t=1}^{T}L_t^{2}}}\right)\cdot
\frac{\frac{252}{T}\sum_{t=1}^{T}L_t r_t}
{\sqrt{\frac{1}{T}\sum_{t=1}^{T}DD_t^{2}}}
$$

其中 $T$ 为评估样本长度（天数 / bar 数）。空仓日 $L_t=0$ 时，对分子累加没有贡献，但对分母的“时间长度 $T$”仍有影响（全日历评估）。

### 4.2 公式参数含义

- $t$：评估日序号（从 $1$ 到 $T$）。
- $T$：评估样本长度（阈值搜索：该段样本天数；walk-forward：OOS 点数）。
- $r_t$：下一期收益（CSV 中 `next_return`），由收盘价计算：$r_t=\frac{close_{t+1}}{close_t}-1$。
- $L_t$：第 $t$ 天仓位/杠杆（当前口径下等于 $b_{\mathrm{raw},t}$；CSV 中 `b_raw` 与 `leverage`），范围 $[-L_{\max},+L_{\max}]$，空仓日为 $0$。
- $L_{\max}$：最大杠杆上限（命令行 `--max_leverage`）。
- $DD_t$：第 $t$ 天回撤（drawdown，CSV 中 `drawdown`），当权益跌破 0 时允许 $DD_t>1$，且
  $$
  E_t = 1+\sum_{i=1}^{t} L_i r_i,\qquad
  DD_t=\frac{\max_{0\le u\le t}E_u - E_t}{\max_{0\le u\le t}E_u}
  $$
- $\sum_{t=1}^{T}L_t r_t$：样本期内累计单利收益（线性累加的策略日收益）。
- $\sum_{t=1}^{T}L_t^2$：仓位能量/暴露强度的平方和（用于风险对齐/归一化）。
- $\sum_{t=1}^{T}DD_t^2$：回撤序列平方和（RMS 形式惩罚回撤强度）。
- $trade\_days$：开仓天数（代码里统计为 $L_t\neq 0$ 的天数），用于“年均交易日 ≥ 50”等约束。
- $Final_{\text{simple}}$：因子评估值，越大表示在相同回撤尺度下的年化收益越高。

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
