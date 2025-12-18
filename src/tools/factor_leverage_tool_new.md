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
- 输出根目录默认 `output/factor_leverage_YYMMDDHHMMSS/`（每次运行生成一个新目录），每个因子一个子目录；
- 每个 code 输出：
  - `leverage_<code>_<factor>.csv`
  - `distribution_<code>_<factor>.png`（仅 Windows 绘制；且仅当训练集得分优于基准时输出；2×3 图：训练集分布（因子/b_raw/leverage）+ train/val/test 收益对比）
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
    [--train_pct 0.6] [--val_pct 0.2] [--test_pct 0.2]
    [--train N] [--val N] [--test N]
    [--zwin 250]
```

说明：

- `--factor` 为空或 `all` 表示跑全部内置因子；也可传多个逗号分隔的名字。
- `--codes` 为空表示使用 data_dir 下全部 code。
- `--D` 用于样本裁剪：若指定则仅使用最近 `D` 天数据；未指定则使用全样本。
- 默认按比例切分全样本：`--train_pct/--val_pct/--test_pct`（三者和为 1）。
- 也可用固定长度切分：`--train/--val/--test`（天数 / bar 数；当提供任意一个 count 参数时启用该模式）。
- `--zwin` 为 rank→normal 的滑窗长度（在线高斯化窗口）。
- `--parquet_dir` 默认直读 parquet（不落盘 CSV），支持两类 schema：
  - 列名为 `icode/tradeDate/pxclose...`
  - 列名为数字字符串：`13`=Wind代码, `9`=交易日期, `3000002/3/4/5`=OHLC, `3000006`=涨跌, `3000007`=涨跌幅, `3000008`=成交量, `3000009`=成交额

## 4. 阈值/杠杆逻辑

1. 对 `factor_raw` 做中位数去中心化；
2. 在线滑窗 rank→normal（窗口 `--zwin`）得到 $z$ 序列；
3. 在 $z$ 空间网格搜索阈值 $\theta$，最大化  
   $$
   Final_{\text{simple}}
   =\left(\sqrt{\frac{T}{\sum_{t=1}^{T}L_t^{2}}}\right)\cdot
   \frac{\frac{252}{T}\sum_{t=1}^{T}L_t r_t}
   {\sqrt{\frac{252}{T}\sum_{t=1}^{T}DD_t^{2}}}
   $$
   并且要求年均交易日 ≥ 50（按 252 交易日/年折算）；
4. 依据最佳 $\theta$ 生成原始信号 $b_{\mathrm{raw}} \in \{0, \pm[1,2]\}$；
5. 生成最终杠杆 $L_t = c_{\mathrm{align}}\,b_{\mathrm{raw},t}$（当前口径下不裁剪）；
6. 通过 $p_{\mathrm{high}}=\Phi(\theta)$ 的经验分位数反推出原值阈值 $x_{\mathrm{high}}/x_{\mathrm{low}}$ 并输出。

### 4.0 训练集 / 验证集 / 测试集（按时间顺序）

本工具按“时间顺序”进行三段切分，避免使用未来信息：

- **训练集（train）**：用于分析因子分布、估计 $z_{\mathrm{cap}}$、确定候选交易模式，并在训练集内部计算原值阈值 $x_{\mathrm{low}}/x_{\mathrm{high}}$。
- **验证集（val）**：用于在候选模式下网格搜索 $\theta$，以验证集上的 $Final_{\text{simple}}$ 选择当期最优 policy（$\theta$/mode/polarity 等）。
- **测试集（test）**：仅用于最终评估（不参与阈值选择）。

对应关系：

- 默认按比例切分全样本：
  - `train_size = floor(train_pct * T)`
  - `val_size = floor(val_pct * T)`
  - `test_size = T - train_size - val_size`
- 输出与评分口径：
  - `leverage_<code>_<factor>.csv` 包含 train/val/test 三段，并在 `split` 列标注 `train/val/test`
- `best_thresholds.csv` 汇总 train/val/test 的 `score/baseline_score` 与时间区间
- `distribution_*.png`：三张直方图使用训练集样本；下排三张折线图分别为 train/val/test 策略 vs 满仓的累计收益对比（单利）
- 仅当训练集上 $Final_{\text{simple}}$ 严格大于训练集基准得分时才会输出 `distribution_*.png`
- 图底部仅展示：因子交易阈值（$\theta$ 及原值阈值）与验证/测试集 `score/baseline` 对比

### 4.1 因子评估值（`score`/`val_score`）最终公式

工具里“因子评估值”对应 `score`（阈值搜索内）/验证集 `val_score`，计算口径一致。设日收益为 $r_t=\mathrm{next\_return}_t$，最终杠杆为 $L_t$（即输出 CSV 中的 `leverage`），则最终公式为：

$$
Final_{\text{simple}}
=\left(\sqrt{\frac{T}{\sum_{t=1}^{T}L_t^{2}}}\right)\cdot
\frac{\frac{252}{T}\sum_{t=1}^{T}L_t r_t}
{\sqrt{\frac{252}{T}\sum_{t=1}^{T}DD_t^{2}}}
$$

其中 $T$ 为评估样本长度（天数 / bar 数，等价于代码里的 $D$）。空仓日 $L_t=0$ 时，对分子累加没有贡献，但对分母的“时间长度 $T$”仍有影响（全日历评估）。

备注：`leverage` 由 `b_raw` 做风险对齐得到，窗口长度取 $\min(250,\text{可用训练样本长度})$，当前口径下不再裁剪上限/下限。

### 4.2 公式参数含义

- $t$：评估日序号（从 $1$ 到 $T$）。
- $T$：评估样本长度（本工具输出的 train/val 点数）。
- $r_t$：下一期对数收益（CSV 中 `next_return`），由收盘价计算：$r_t=\log\left(\frac{close_{t+1}}{close_t}\right)$。
- $b_{\mathrm{raw},t}$：阈值映射后的原始仓位幅度（CSV 中 `b_raw`），范围约为 $\{0,\pm[1,L_{\max}]\}$。
- $L_t$：第 $t$ 天最终杠杆（CSV 中 `leverage`），由风险对齐系数 $c_{\mathrm{align}}$ 得到：$L_t=c_{\mathrm{align}}\,b_{\mathrm{raw},t}$（当前口径下不裁剪）。
- $L_{\max}$：最大杠杆上限（命令行 `--max_leverage`）。
- $DD_t$：第 $t$ 天回撤（drawdown，CSV 中 `drawdown`），当前口径为“绝对回撤”，且
  $$
  E_t=\sum_{i=1}^{t} L_i r_i,\qquad
  DD_t=\max_{0\le u\le t}E_u - E_t
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
