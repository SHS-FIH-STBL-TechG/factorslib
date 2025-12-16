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
3. 在 $z$ 空间网格搜索阈值 $\theta$，最大化  
   $$
   score=
   \frac{\prod_{t=1}^{T}\left(1+L_t r_t\right)-1}
   {\sqrt{\frac{1}{D}\sum_{t=1}^{T}\left(DD_t\right)^2}}
   $$
   并且要求年均交易日 ≥ 50（按 252 交易日/年折算）；
4. 依据最佳 $\theta$ 生成原始信号 $b_{\mathrm{raw}} \in \{0, \pm[1,2]\}$；
5. 依据风险对齐和最大杠杆上限得到最终杠杆：
   $$L_t=\mathrm{clamp}(c\cdot b_{\mathrm{raw},t},[-L_{\max},+L_{\max}])$$
6. 通过 $p_{\mathrm{high}}=\Phi(\theta)$ 的经验分位数反推出原值阈值 $x_{\mathrm{high}}/x_{\mathrm{low}}$ 并输出。

### 4.1 因子评估值（`score`/`oos_score`）最终公式

工具里“因子评估值”对应 `score`（阈值搜索内）/`oos_score`（walk-forward OOS 评估），计算口径一致。设日收益为 $r_t=\mathrm{next\_return}_t$，最终杠杆为 $L_t$（即输出 CSV 中的 `leverage`），则最终公式可直接写为（不引入中间变量）：

$$
score=
\frac{\prod_{t=1}^{T}\left(1+L_t r_t\right)-1}
{\sqrt{\frac{1}{D}\sum_{t=1}^{T}\left(DD_t\right)^2}}
$$

其中 $D$ 为分母归一化天数（阈值搜索用 `--D`/样本天数；walk-forward OOS 用 $D=T$=OOS 点数）。

备注：空仓日 $L_t=0\Rightarrow (1+L_t r_t)=1$，因此分子连乘“写到 $T$”只是为了和全日历回撤对齐；数值上等价于只在开仓日（$L_t\neq 0$）做连乘。

### 4.2 公式参数含义

- $t$：评估日序号（从 $1$ 到 $T$）。
- $T$：评估样本长度（阈值搜索：该段样本天数；walk-forward：OOS 点数）。
- $r_t$：下一期收益（CSV 中 `next_return`），由收盘价计算：$r_t=\frac{close_{t+1}}{close_t}-1$。
- $L_t$：第 $t$ 天最终杠杆（CSV 中 `leverage`），范围 $[-L_{\max},+L_{\max}]$，空仓日为 $0$。
- $L_{\max}$：最大杠杆上限（命令行 `--max_leverage`）。
- $DD_t$：第 $t$ 天回撤（drawdown，CSV 中 `drawdown`），取值 $[0,1]$，且
  $$
  DD_t
  =1-\frac{\prod_{i=1}^{t}\left(1+L_i r_i\right)}{\max_{1\le u\le t}\prod_{i=1}^{u}\left(1+L_i r_i\right)}
  $$
- $D$：回撤均方根的归一化分母天数（阈值搜索：`--D`/样本天数；walk-forward：$D=T$）。
- $trade\_days$：开仓天数（代码里统计为 $L_t\neq 0$ 的天数），用于“年均交易日 ≥ 50”等约束；不改变分子连乘的数值（空仓日乘子为 $1$）。
- $score$：因子评估值，越大表示在相同回撤尺度下的累计收益越高。

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
