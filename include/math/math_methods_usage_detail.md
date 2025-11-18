# RollWinOp 数学库使用手册（详细版）

---

## 0. 准备工作：如何在工程里使用数学库

### 0.1 CMake / 工程配置

在你自己的 CMake 里，只要把 RollWinOp 项目（包含 `math/`、`utils/`、`third_party/`）作为一个 include 目录即可：

```cmake
# 假设 RollWinOp 源码在 path/to/RollWinOp_patched
include_directories(
    path/to/RollWinOp_patched
    path/to/RollWinOp_patched/third_party/eigen
    path/to/RollWinOp_patched/third_party/boost
    path/to/RollWinOp_patched/third_party/spdlog/include
    path/to/RollWinOp_patched/third_party/nanoflann
)
```

之后在任意 cpp 里直接：

```cpp
#include "math/statistics.h"
#include "math/sliding_statistics.h"
#include "math/fractional_diff.h"
#include "math/rolling_autocorr.h"
#include "math/rolling_entropy.h"
#include "math/modwt.h"
#include "math/online_pca.h"
#include "math/symbolic_dynamics.h"
#include "math/vector_field_analysis.h"
#include "math/hawkes_intensity.h"
#include "math/info_geometry.h"
#include "math/memory_kernel.h"
#include "math/persistent_h0.h"
#include "math/tda_rips_h1.h"
#include "math/rosenstein_lyapunov.h"
#include "math/fnn.h"
#include "math/incremental_rank.h"
#include "math/bad_value_policy.h"
#include "utils/nn/ann_index.h"
```

> 实际项目里你不需要全 include，只引入自己要用的模块即可。

---

## 1. 坏值策略：`math/bad_value_policy.h`

绝大部分「滑窗类模板」都有一个模板参数 `BadValuePolicy`，用来控制 NaN、Inf、极端异常值的处理策略。

典型形态（伪代码）：

```cpp
template<typename T, typename BadValuePolicy = bad_value::NoOpPolicy<T>>
class SlidingWindowStats {
    BadValuePolicy bad_policy_;
    // ...
};
```

### 1.1 有哪些策略？


1. **NoCheckBadValuePolicy（默认）**
  - 什么都不做，只是把原始值原样丢进后端计算；
  - 统计函数内部可能再单独对 NaN 做过滤（如 `Statistics`）。

2. **SkipNaNInfPolicy**
  - 检测到 `std::isnan(x)` 则在当前窗口统计中直接忽略；
  - 用于希望“坏值不参与统计，但也不影响窗口长度”的场景。

3. **ZeroNaNInfPolicy**
  - 检测到坏值时，用某个安全值替换（0、均值、上一时刻值等）。

> 默认你只要**什么都不传**，就走“不处理”的策略：

```cpp
SlidingWindowStats<double> sw(64);  // 默认 bad_value::NoCheckBadValuePolicy<double>
```

除下面特殊说明外，只有在你明确希望“自动过滤坏值”时，才显式带策略模板参数。

---

## 2. 一次性统计：`math/statistics.h`

### 2.1 我只想快速算个均值/方差/相关

```cpp
#include "math/statistics.h"

using factorlib::math::Statistics;

void simple_stats_example() {
    std::vector<double> x{1.0, 2.0, std::numeric_limits<double>::quiet_NaN(), 4.0};

    double m  = Statistics<double>::mean(x);       // 跳过 NaN，得到 (1+2+4)/3 = 7/3
    double sd = Statistics<double>::stddev(x);     // 同样只对非 NaN 计算
}
```

支持的典型函数（以模板 `T` 为数值类型）：

- `mean(container)`
- `stddev(container)`
- `quantile(container, percentile)`  // 0~1
- `median(container)`
- `median_rank(container, value)`
- `correlation(x_container, y_container)`
- `covariance(x_container, y_container)`
- `rolling_mean(container, window_size)`  // 一次性滚动平均（非在线）

**容器要求：**

- 只要有 `begin() / end() / size()`，并且元素是算术类型即可：
  - `std::vector<T>`
  - `std::deque<T>`
  - `std::array<T,N>`
  - 甚至你自己的容器，只要接口兼容。

### 2.2 坏值处理与返回值约定

- 对于 NaN：
  - 会跳过，并打印 warn 日志，形式类似：
    > `Statistics::mean: skipped X NaN values out of N`
- 对于“有效样本数不足”的情况：
  - 均值：返回 `0.0`；
  - 方差 / 协方差 / 相关：如果有效样本 < 2，则返回 `0.0`；
  - 分位数 / 中位数：如果全部是 NaN，则返回 `0.0`。

### 2.3 时间复杂度怎么优化的？

**朴素方案**：

- 计算方差时，如果先算均值再二次遍历：
  - 均值：`O(N)`
  - 再次遍历求平方和：`O(N)`
  - 总体也是 `O(N)`，但数值稳定性差（大数据时容易精度损失）。

**当前实现要点：**

- 均值仍然是单遍 `O(N)`；
- 方差使用 **Welford/Pébay 单次遍历算法**：
  - 每来一个 `x`，更新：
    ```text
    n  += 1
    delta  = x - mean
    mean  += delta / n
    m2    += delta * (x - mean)   // 累积二阶矩
    ```
  - 最后 `variance = m2 / (n-1)`；
- 好处：
  - **一次遍历**：只扫一遍数据，`O(N)`；
  - **数值更稳定**：避免“先算大和，再减去 N*m²”的差分放大误差。

> 对于 `quantile` / `median`：
> - 需要排序，不能再优化到线性，复杂度就是 `O(N log N)`，这是正常上限。

---

## 3. 滑动统计：`math/sliding_statistics.h`

### 3.1 我只想做滑窗均值/方差

典型使用方式：

```cpp
#include "math/sliding_statistics.h"

using factorlib::math::SlidingWindowStats;

void sliding_example() {
    const std::size_t W = 4;
    SlidingWindowStats<double> sw(W);  // 默认坏值策略：不处理

    std::vector<double> xs{1, 2, 3, 4, 5, 6};

    for (double x : xs) {
        sw.push(x);

        if (!sw.ready()) {
            // 还没满 W 个样本
            continue;
        }

        double m   = sw.mean();      // 当前滑窗均值
        double var = sw.variance();  // 当前滑窗方差
        double sse = sw.sse();       // 平方和
        // 你可以在这里构造因子
    }
}
```



### 3.2 时间复杂度与优化思路

朴素滑窗计算方式：**每个时间步重新在窗口上扫一遍**：

- 对窗口 `{x_{t-W+1}, ..., x_t}` 每次算均值：`O(W)`；
- 一共 `N` 个时间步，总复杂度：`O(N·W)`，高频下非常慢。

**当前实现的优化：**

- 维护窗口的：
  - `sum = Σ x_i`
  - `sum_sq = Σ x_i²`
- 每步只做增量更新：
  - `sum += x_new - x_old;`
  - `sum_sq += x_new² - x_old²;`
- 然后立刻可以得到：
  - `mean = sum / W;`
  - `var = (sum_sq - sum²/W) / (W-1);`
- 每步只做 **常数次加减乘除**：**`O(1)`**。

所以：

- **时间复杂度从 `O(N·W)` 降到 `O(N)`**；


---

## 4. 滑动秩与中位秩：`math/incremental_rank.h`

### 4.1 如何用它做中位秩因子？

```cpp
#include "math/incremental_rank.h"

using factorlib::math::IncrementalRank;

void rank_example() {
    const std::size_t W = 5;
    IncrementalRank<double> rank(W);

    std::vector<double> xs{1, 3, 2, 5, 4, 10, -1};

    for (double x : xs) {
        rank.push(x);

        if (!rank.ready()) continue;  // 样本数 < W 时不计算

        // 例如：以当前值本身为查询，得到它在窗口中的中位秩
        double r = rank.median_rank(x);

        // 也可以拿窗口最后一个值 / 中位数等作 rank
    }
}
```

### 4.2 时间复杂度优化思路

**朴素方案**：

- 每个时间步维护一个长度为 `W` 的数组；
- 需要秩 / 中位数时就 `std::sort` 一下：`O(W log W)`；
- 高频下 `N` 步总复杂度 `O(N·W log W)`，几乎不可用。

**优化结构**：

- 使用 **有序数据结构**（如平衡树、多重集等）保存窗口内容；
- 插入一个元素 / 删除最旧元素：`O(log W)`；
- 查询中位数 / 计算 rank：`O(1)` 或 `O(log W)`（视实现）。

总体复杂度：

- 每步 `O(log W)`，总 `O(N log W)`；
- 对于常见窗口大小（几百几千）性能足够。

---

## 5. 分数阶差分：`math/fractional_diff.h`

### 5.1 快速上手：分数阶差分因子

```cpp
#include "math/fractional_diff.h"

using factorlib::math::FractionalDifferentiatorGL;

void frac_diff_example() {
    double alpha = 0.7;   // 分数阶，0~1 常用
    std::size_t L = 20;   // GL 权重长度（记忆长度）

    FractionalDifferentiatorGL<double> fd(alpha, L);

    std::vector<double> xs{/* 时间序列 */};

    for (double x : xs) {
        fd.push(x);
        if (!fd.ready()) continue;  // 样本数不足 L+1 时还没有输出

        double d = fd.value();      // 当前时刻的分数阶差分
        // 用 d 构造你的高频因子
    }
}
```

### 5.2 时间复杂度与优化

**数学形式**：GL 分数阶差分：

$$
D^{\alpha} x_t \approx \sum_{k=0}^{L} w_k x_{t-k}
$$

朴素做法：

- 每个时间步重算权重 `w_k`，再做长度 L 的卷积：`O(L)` + `O(L)`；
- 若每次都“从公式重新算权重”，构造开销也大。

当前实现优化点：

1. **预计算权重**：构造时一次性算出 `w_k`，复杂度 `O(L)`，之后重用；
2. **滑动累积**：每步只是执行固定长度 L 的内积：
  - `value = Σ w_k x_{t-k}`，复杂度 `O(L)`；
3. **与窗口大小无关**：只跟记忆长度 `L` 有关，通常 `L` 比窗口 `W` 小。

总的来看：

- 构造：一次 `O(L)`；
- 每步：`O(L)`；
- 如果你的时间序列很长、但分数阶记忆只取几十个点，是可以接受的。

---

## 6. 滑动自相关：`math/rolling_autocorr.h`

### 6.1 用法示例

```cpp
#include "math/rolling_autocorr.h"

using factorlib::math::RollingAutoCorr;

void autocorr_example() {
    std::size_t W   = 64;  // 窗口
    std::size_t lag = 1;   // 滞后阶数

    RollingAutoCorr<double> rac(W, lag);

    std::vector<double> xs{/* 时间序列 */};

    for (double x : xs) {
        rac.push(x);
        if (!rac.ready()) continue;

        double rho = rac.value();  // 当前窗口的自相关系数
    }
}
```

### 6.2 时间复杂度优化

朴素方案：

- 每次窗口平移，就取 `W` 个点，手工算相关系数：
  - 均值 `O(W)`
  - 协方差 & 方差 `O(W)`
  - 总 `O(W)` 每步，整体 `O(N·W)`。

当前实现优化：

- 维护窗口内的各种和：
  - `Σ x_t`, `Σ x_t²`, `Σ x_t x_{t-lag}` 等；
- 窗口平移时：
  - 再加上新样本的贡献；
  - 减去旧样本的贡献；
- 用这些和 **一次性算出** 协方差 / 方差，从而得到相关。

每步只做常数次运算 → `O(1)`，总 `O(N)`。

---

## 7. 滑动熵：`math/rolling_entropy.h`

### 7.1 快速使用（直方图熵）

```cpp
#include "math/rolling_entropy.h"

using factorlib::math::RollingEntropy;

void entropy_example() {
    std::size_t W = 128;
    std::vector<double> edges{0, 10, 20, 30, 40};  // 分箱边界

    RollingEntropy<double> rent(W, edges);

    std::vector<double> xs{/* 序列 */};
    for (double x : xs) {
        rent.push(x);
        if (!rent.ready()) continue;

        double H = rent.entropy();  // 当前窗口的 Shannon 熵
    }
}
```

### 7.2 时间复杂度优化

朴素方案：

- 每步都重新在窗口上 `O(W)` 遍历，根据分箱统计频数，再算熵：
  - `count[i] / W` 得到概率；
  - `H = - Σ p_i log p_i`；
- 总复杂度 `O(N·W)`。

当前实现优化：

- 维护一个长度为 `B` 的计数数组 `count[0..B-1]`：
  - 新样本到来：找到落在哪个 bin，`count[bin]++`；
  - 旧样本滑出：`count[bin_old]--`；
- 每步只需：
  - 更新两个计数；
  - 缓存/重算一次 `Σ p_i log p_i` （按实现可能是 `O(B)`）。

如果 `B` 是常数（比如 10~50 个分箱）：

- 每步复杂度约 `O(B)`，可以认为是常数；
- 总体 `O(N)` 或 `O(N·B)`。

---

## 8. MODWT（小波）：`math/modwt.h`

### 8.1 快速使用：趋势能量比

```cpp
#include "math/modwt.h"

using factorlib::math::RollingMODWT;

void modwt_example() {
    std::size_t W = 128;
    int levels    = 3;          // 分解层数
    std::string wavelet = "haar";   // 具体以实现中定义为准

    RollingMODWT<double> rm(W, levels, wavelet);

    std::vector<double> xs{/* 序列 */};
    for (double x : xs) {
        rm.push(x);
        if (!rm.ready()) continue;

        double ratio = rm.trend_energy_ratio();
    }
}
```

### 8.2 时间复杂度优化

MODWT 本质是若干小波滤波器的卷积；朴素做法：

- 每次窗口平移都对窗口做完整 MODWT：
  - 每层一个长度 L 的滤波器，`O(W·L)`；
  - 多层叠加，`O(W·L·J)`。

当前实现思路：

- 通过“增量卷积”或“内部缓冲”方式，**只在新增样本附近更新系数**：
  - 对每一层，维护一段“卷积缓存”；
  - 滑窗时，剔除旧贡献、加入新贡献；
- 这样每步复杂度近似为 `O(J)` 或 `O(J·L_const)`，不再和 `W` 成正比。

---

## 9. Rosenstein Lyapunov：`math/rosenstein_lyapunov.h`

### 9.1 我想估计序列是否混沌

```cpp
#include "math/rosenstein_lyapunov.h"

using factorlib::math::rosenstein_lyapunov;

void lyapunov_example() {
    std::vector<double> x{/* 标量时间序列 */};

    int m       = 2;     // 嵌入维度
    int tau     = 2;     // 延迟
    int theiler = 10;    // Theiler 窗口，排除时间上太近的点
    int fit_min = 3;
    int fit_max = 15;

    double lambda = rosenstein_lyapunov(x, m, tau, theiler, fit_min, fit_max);
}
```

### 9.2 时间复杂度优化点

**核心步骤：**

1. 相空间重构：`Y_m = embed(x, m, tau)` → `O(N·m)`；
2. 最近邻搜索：对于每个 `Y_m(i)` 找最近邻 `j(i)`；
3. 随时间推移计算 `log` 距离，并线性拟合斜率。

朴素最近邻搜索：两层循环：

- 对每个点 `i`，扫一遍所有其他点 `j`，计算距离：`O(N)`；
- 总体 `O(N² · m)`，序列稍长就不可用。

当前实现使用：

- `utils/nn/ann_index.h` 里的 **KDTreeANN（基于 nanoflann）**：
  - 构建 KD-tree：`O(N log N)`；
  - 每个点查询最近邻：`O(log N)`；
- 整体近似：`O(N log N · m)`。

---

## 10. 虚假最近邻 FNN：`math/fnn.h`

### 10.1 如何判断嵌入维度够不够？

基于 `KDTreeANN`，使用方式大致如下：

```cpp
#include "math/fnn.h"

using factorlib::math::fnn_ratio;

void fnn_example() {
    std::vector<double> x{/* 序列 */};
    int tau     = 2;
    double Rtol = 10.0;

    double r1 = fnn_ratio(x, 1, tau, Rtol);   // m = 1
    double r2 = fnn_ratio(x, 2, tau, Rtol);   // m = 2
    double r3 = fnn_ratio(x, 3, tau, Rtol);   // m = 3

    // 通常：r1 > r2 > r3，且当 m 足够大时，r_m 会降到某个较低水平
}
```

### 10.2 时间复杂度优化点（你当前版本）

参见你代码中的注释：

- 嵌入构造：`O(N·m)`；
- KD-Tree 构建：`O(N log N)`；
- 每个点最近邻：`O(log N)`，总体 `O(N log N)`；
- 总体：`O(N·m + N log N)`。

相比朴素暴力搜索 `O(N²·m)`，对于 N 上千上万时，差异非常明显。

---

## 11. 信息几何：`math/info_geometry.h`

### 11.1 Bernoulli Fisher 信息

```cpp
#include "math/info_geometry.h"

using factorlib::math::fisher_bernoulli;

void fisher_example() {
    double theta = 0.3;  // Bernoulli 成功概率
    double I = fisher_bernoulli(theta);  // 理论上 ~ 1/(theta*(1-theta))
}
```

- θ 在 (0,1) 内：返回有限正数；
- θ=0 或 1：返回 `+inf`，表示在边界处信息量无限大（理论性质）。

### 11.2 多项式 Fisher 对角近似

```cpp
#include "math/info_geometry.h"

using factorlib::math::fisher_multinomial_diag;

void fisher_multi_example() {
    std::vector<double> p{0.2, 0.3, 0.5};
    auto diag = fisher_multinomial_diag(p);  // 每个分量大致 ~ 1/p[i]
}
```

复杂度都为 `O(K)`，其中 `K` 为类别数。

---

## 12. 符号动力学与熵：`math/symbolic_dynamics.h`

### 12.1 快速用法：从数值序列到符号熵

```cpp
#include "math/symbolic_dynamics.h"

using factorlib::math::Symbolizer;
using factorlib::math::RollingSymbolicDynamics;

void sym_dyn_example() {
    // 先定义数值 -> 符号 的映射：edges 分成若干区间
    std::vector<double> edges{0, 1, 2, 3};   // (0,1], (1,2], (2,3]
    Symbolizer<double> sym(edges);

    std::size_t W = 128;
    RollingSymbolicDynamics<double> rsd(W, sym);

    std::vector<double> xs{/* 序列 */};
    for (double x : xs) {
        rsd.push(x);
        if (!rsd.ready()) continue;

        auto P     = rsd.transition_matrix();      // S×S 概率矩阵
        double h   = rsd.topological_entropy();    // 拓扑熵
        double sig = rsd.entropy_production_rate();// 熵产生率
    }
}
```

### 12.2 时间复杂度

- 每步更新：
  - 符号化：`O(1)`（二分查找边界或简单比较）；
  - 更新某条转移计数：`O(1)`；
- 计算熵相关量：
  - 对 S×S 矩阵做一次自旋：`O(S²)`（S 为符号数）；
  - 通常 S 不大（3~10），可视为常数。

整体上：**每步 `O(1)` 或 `O(S²)`**，不依赖窗口长度 `W`。

---

## 13. 在线 PCA：`math/online_pca.h`

### 13.1 如何对流数据做 PCA？

```cpp
#include "math/online_pca.h"

using factorlib::math::OnlinePCA;

void online_pca_example() {
    int dim = 3;      // 原始特征维度
    int k   = 2;      // 主成分数
    double lr = 0.05; // 学习率

    OnlinePCA<double> pca(dim, k, lr);

    // 假设我们有很多 3 维样本：
    std::vector<std::vector<double>> samples = {
        {1.0, 0.5, -0.2},
        {0.9, 0.4, -0.1},
        // ...
    };

    for (const auto& v : samples) {
        pca.push(v);  // 每次来一个样本就更新一次主方向
    }

    auto comps = pca.components();  // k × dim 的矩阵（向量组）

    // 如果想评估最近一段样本的解释方差：
    auto var = pca.explained_variance(samples);
}
```

### 13.2 时间复杂度优化

**朴素 PCA**：

- 收集所有 `N` 个样本，形成 `N×d` 矩阵；
- 计算协方差矩阵 `d×d`：`O(N·d²)`；
- eigen / SVD：`O(d³)`；
- 对于流式高频数据，不能每次 tick 重跑一次。

**在线 PCA（Oja/SGD 风格）**：

- 只维护一个 `d×k` 的主向量矩阵；
- 每个样本 `(d 维)` 到来时：
  - 做一次投影和梯度更新：`O(d·k)`；
- 总复杂度：`O(N·d·k)`，并且是**在线**的。

---

## 14. 记忆核与 Hawkes：`math/memory_kernel.h`、`math/hawkes_intensity.h`

### 14.1 Hawkes 强度：`HawkesIntensity`

```cpp
#include "math/hawkes_intensity.h"

using factorlib::math::HawkesIntensity;

void hawkes_example() {
    double mu    = 0.1;  // 基线强度
    double alpha = 0.3;  // 自激系数
    double beta  = 1.2;  // 衰减速度
    double dt    = 1.0;  // 时间步长

    HawkesIntensity H(mu, alpha, beta, dt);

    std::vector<int> events = {1,0,0,1,0,0};
    for (int n_t : events) {
        double lambda_t = H.update(n_t);
    }
}
```

时间复杂度：每步只做 **常数次加法/乘法**，`O(1)`。

### 14.2 记忆核估计：`MemoryKernelEstimator`

```cpp
#include "math/memory_kernel.h"
using factorlib::math::MemoryKernelEstimator;
```

**用途：**

- 用一个有限长窗口上的自相关函数 $ \rho(k) $ 去近似一个“记忆核”
  $$
  M(\tau) \approx \sum_{k=1}^{L} w_k(\alpha)\, \rho(k)
  $$
- 其中：
  - $ w_k(\alpha) $ 来自 Grünwald–Letnikov 分数阶权重（`FractionalDifferentiatorGL`）；
  - $ \rho(k) $ 由 `RollingAutoCorr` 给出（同一条原始时间序列，滞后阶分别为 1..L）；
  - 常数序列时，`RollingAutoCorr` 在方差为 0 的情况下返回 0，因此 $ M(\tau) \approx 0 $。


---

#### 接口与用法

```cpp
// 模板参数：
//   T         数值类型（double 等）；
//   BadPolicy 坏值处理策略（参考 math/bad_value_policy.h）
// 一般可以直接用默认的 NoCheckBadValuePolicy（不做额外坏值处理）。
template<typename T = double, typename BadPolicy = NoCheckBadValuePolicy>
class MemoryKernelEstimator {
public:
    MemoryKernelEstimator(std::size_t W, std::size_t L, double alpha);

    bool   push(T x);    // 喂入一个新样本
    bool   ready() const;// 是否已有足够窗口长度
    double value() const;// 当前记忆核估计值
};
```

构造参数含义：

- `W`：窗口长度（Sliding 自相关所用的窗口大小）；
- `L`：记忆阶数，即考虑的最大滞后阶数；
- `alpha`：分数阶参数，用于生成 `w_k(α)` 权重。

约束：

- 需要满足 `L + 1 < W`，保证每个滞后阶都能在窗口内形成足够多的配对样本；  
  否则构造函数会抛 `std::invalid_argument`。

典型用法：

```cpp
std::size_t W = 64;   // 自相关窗口
std::size_t L = 5;    // 记忆核最高阶
double alpha = 0.6;   // 分数阶参数

MemoryKernelEstimator<double> mk(W, L, alpha);

// 在线喂入时间序列
for (double x : data_stream) {
    mk.push(x);
    if (mk.ready()) {
        double M = mk.value();
        // 在这里使用记忆核：比如做监控 / 因子输入等
    }
}
```

---

#### 坏值处理（BadValuePolicy）

- `MemoryKernelEstimator` 本身不直接判断 NaN，而是**把原始值原样传给 `RollingAutoCorr`**；
- `RollingAutoCorr<T, BadPolicy>` 会根据 `BadPolicy` 决定：
  - 是否接受该点；
  - 是否跳过 NaN / 非有限值；
  - 是否在有坏值时记日志。
- 你可以在 `math/bad_value_policy.h` 里选择合适策略，一般默认 `NoCheckBadValuePolicy` 就够用。

---

#### 时间复杂度与增量特性

- **增量性：✅**
  - 每次 `push(x)` 仅更新 `L` 个自相关估计器（滞后 1..L），不需要重扫整段历史；
  - 所有权重 `w_k(α)` 由 `FractionalDifferentiatorGL` 预先计算并缓存。

- **单次更新复杂度：**
  - 每个 lag 对应一个 `RollingAutoCorr`，更新一次是 O(1)；
  - 共 `L` 个 lag ⇒ 每次 `push` 为 **O(L)**。

- **取值复杂度：**
  - `value()` 只是：
    $$
    M(\tau) = \sum_{k=1}^{L} w_k(\alpha)\, \rho(k)
    $$
    逐项相乘加和 ⇒ **O(L)**。

综上，`MemoryKernelEstimator` 适合：

- 高频因子里用固定窗口估计“长期记忆强度”；
- 不想每个时刻都重算全量自相关，而是希望**严格 O(L) 增量更新**。
---

## 15. 向量场分析：`math/vector_field_analysis.h`

### 15.1 如何计算散度与旋度？

```cpp
#include "math/vector_field_analysis.h"

using factorlib::math::Grid2DSize;
using factorlib::math::VectorField2D;
using factorlib::math::divergence;
using factorlib::math::curl_z;

void vf_example() {
    int nx = 64, ny = 64;
    Grid2DSize g{nx, ny};

    VectorField2D F;
    F.Fx.assign(nx*ny, 0.0);
    F.Fy.assign(nx*ny, 0.0);

    // 填充向量场 (Fx(x,y), Fy(x,y))
    // ...

    auto div = divergence(F, g);
    auto cz  = curl_z(F, g);
}
```

时间复杂度：

- 每个网格点做一个常数规模的有限差分计算；
- 总体 `O(nx·ny)`，没有更多可优化空间。

---

## 16. 持续同调：`math/persistent_h0.h`、`math/tda_rips_h1.h`

### 16.1 H0：连通分量

典型流程（概念）：

1. 给定一维 / 二维 / 多维点集；
2. 选一组半径阈值；
3. 对每个半径，逐步把“距离小于半径”的点连接起来；
4. 记录连通块的“出生-死亡半径”，并计算平均持久度。

实现细节：

- 内部一般使用 **并查集（Union-Find）** 来动态维护连通块；
- 对每一条边的“激活”操作是 `O(α(N))`（几乎常数），总体近似 `O(E)`；
- 使用稀疏邻接（只保留近邻）避免 `O(N²)` 的完全图。

### 16.2 H1：Rips 复形 H1

- 基于点之间距离的 Vietoris–Rips 复形；
- 朴素复杂度为 `O(N³)`（所有三角形组合）；
- 实际实现中通过：
  - 半径阈值；
  - 稀疏邻接 / 最近邻；
  - 降采样；
- 将实际规模控制在可用范围内。

> 这部分更偏“离线分析”，通常窗口长度不宜太大（比如几百个点以内）。

---

## 17. 分布 & 数值工具 & 线性代数

---

### 17.1 概率分布：`math/distributions.h`

头文件：

```cpp
#include "math/distributions.h"
using factorlib::math::Distributions;
```

#### 17.1.1 标准正态分布逆 CDF：`Distributions::normal_quantile`

**功能：**

- 给定概率 `p`（0~1 之间），返回标准正态分布 `N(0,1)` 下的分位点 `z`，即：
  \[
  P(Z \le z) = p, \; Z \sim N(0,1)
  \]

**用法：**

```cpp
double p1 = 0.025;
double p2 = 0.5;
double p3 = 0.975;

double z1 = Distributions::normal_quantile(p1);  // 约 -1.96
double z2 = Distributions::normal_quantile(p2);  // 0
double z3 = Distributions::normal_quantile(p3);  // 约 +1.96
```

**注意事项：**

- 若 `p` 不是有限数（NaN/inf）或不在 `(0,1)`，函数会：
  - 打一条 `LOG_WARN`；
  - 返回 `0.0` 作为退化值。
- 内部使用 Wichura 的近似公式，复杂度 **`O(1)`**。

---

#### 17.1.2 标准正态 CDF：`Distributions::normal_cdf`

**功能：**

- 给定 `z`，返回 `Φ(z) = P(Z ≤ z), Z ~ N(0,1)`。

**函数模板：**

```cpp
template<typename T>
static T normal_cdf(T z);
```

**用法：**

```cpp
double z = 1.0;
double p = Distributions::normal_cdf(z);  // 约 0.8413

float zf = 1.0f;
float pf = Distributions::normal_cdf(zf); // 也可以用 float
```

**注意事项：**

- `T` 必须是浮点类型（float/double/long double）；
- 内部用多项式 + `std::erf` 或近似公式，复杂度 **`O(1)`**。

---

#### 17.1.3 经验逆 CDF：`Distributions::empirical_inverse_cdf`

**功能：**

- 给定一批样本（任意容器）和概率 `probability`，返回**经验分布的分位点**。
- 用在“根据历史收益分布反推阈值”等场景。

**函数模板：**

```cpp
template<typename Container>
static double empirical_inverse_cdf(const Container& data, double probability);
```

要求：

- `Container`：支持 `begin/end/size`；
- `Container::value_type` 必须是算术类型（int/float/double 等）。

**用法示例：**

```cpp
std::vector<double> returns = { -0.01, 0.02, 0.03, -0.02, 0.05 };

double q95 = Distributions::empirical_inverse_cdf(returns, 0.95); // 经验 95% 分位
double q05 = Distributions::empirical_inverse_cdf(returns, 0.05); // 经验 5% 分位
```

**坏值处理：**

- 会自动跳过 NaN：
  - 若存在 NaN：打印一条 warn；
  - 若清洗后没有有效样本：返回 `0.0`。
- 若 `probability` 非有限或不在 `[0,1]`：
  - 打一条 warn；
  - 返回 `0.0`。

**时间复杂度：**

- 清洗 + 拷贝到 `std::vector<double>`：`O(N)`；
- `std::sort` 排序：`O(N log N)`；
- 分位点插值：`O(1)`；
- 总体：**`O(N log N)`**（N 为样本数）。

---

#### 17.1.4 F 检验右尾 p 值：`fisher_f_sf` / `fisher_f_pvalue_right_tail`

这两个函数是做 **F 分布右尾概率**（通常用作 F 检验的 p 值）。

**函数模板：**

```cpp
template<typename TFloat>
inline TFloat fisher_f_sf(TFloat Fobs, int d1, int d2);

template<typename TFloat>
inline TFloat fisher_f_pvalue_right_tail(TFloat Fobs, int d1, int d2);
```

- `Fobs`：观测到的 F 统计量（`>=0`）；
- `d1`：自由度 1（例如受限 vs 非受限模型的参数差 q）；
- `d2`：自由度 2（例如非受限模型残差自由度 N - p - q - 1）。

`fisher_f_pvalue_right_tail` 只是对 `fisher_f_sf` 的别名，更语义化。

**用法示例：**

```cpp
double Fobs = 5.12;
int d1 = 3;
int d2 = 100;

double p = Distributions::fisher_f_pvalue_right_tail(Fobs, d1, d2);
// 若 Boost.Math 存在，则是真实 F 分布右尾 p 值；否则退化为 1.0（保守：永远“不过显著”）
```

**数值与依赖：**

- 若你的编译环境能找到 `boost/math/distributions/fisher_f.hpp`：
  - 使用 Boost.Math 的 `fisher_f` 分布 + `cdf(complement)` 计算右尾概率；
  - 复杂度 `O(1)`（Boost 内部实现）。
- 若找不到 Boost：
  - 函数会打印 warn；
  - 始终返回 `1.0`（保守地认为“不显著”）。

---

### 17.2 数值工具：`math/numeric_utils.h`

头文件：

```cpp
#include "math/numeric_utils.h"
using factorlib::math::NumericUtils;
```

这是一个“单点运算”的工具类，所有方法都是 `static`，不需要实例化。

#### 17.2.1 安全对数：`safe_log`

```cpp
template<typename T>
class NumericUtils {
public:
    static double safe_log(T x);
    // ...
};
```

**语义：**

- 当 `x > 0` 时：返回 `std::log(x)`；
- 当 `x <= 0` 时：返回 `std::log(std::numeric_limits<double>::min())`，避免 NaN。

**用法：**

```cpp
double a = NumericUtils<double>::safe_log(10.0);   // ~ 2.302585
double b = NumericUtils<double>::safe_log(0.0);    // 非 NaN 的一个很小的负大数
double c = NumericUtils<double>::safe_log(-5.0);   // 同样不会 NaN
```

复杂度：单次 **`O(1)`**。

---

#### 17.2.2 安全除法：`safe_divide`

```cpp
static double safe_divide(T numerator, T denominator);
```

**语义：**

- 如果 `denominator == 0`：返回 `0.0`，避免除零；
- 否则返回 `numerator / denominator`。

**用法：**

```cpp
double v1 = NumericUtils<double>::safe_divide(10.0, 2.0);  // 5.0
double v2 = NumericUtils<double>::safe_divide(1.0, 0.0);   // 0.0（安全退化值）
```

复杂度：**`O(1)`**。

---

#### 17.2.3 截断到区间：`clamp`

```cpp
static T clamp(T value, T min_val, T max_val);
```

**语义：**

- 如果 `value < min_val`：返回 `min_val`；
- 如果 `value > max_val`：返回 `max_val`；
- 否则返回 `value` 本身。

**用法：**

```cpp
int x1 = NumericUtils<int>::clamp(5, 0, 10);    // 5
int x2 = NumericUtils<int>::clamp(-3, 0, 10);   // 0
int x3 = NumericUtils<int>::clamp(20, 0, 10);   // 10
```

复杂度：**`O(1)`**。

---

#### 17.2.4 判断接近 0：`is_near_zero`

```cpp
static bool is_near_zero(T value,
                         T epsilon = std::numeric_limits<T>::epsilon());
```

**语义：**

- 返回 `|value| < epsilon`；
- 默认 epsilon 使用该类型的机器精度。

**用法：**

```cpp
bool f1 = NumericUtils<double>::is_near_zero(1e-12);           // true（一般来说）
bool f2 = NumericUtils<double>::is_near_zero(1e-3);            // false
bool f3 = NumericUtils<double>::is_near_zero(1e-3, 1e-2);      // true（自定义 epsilon）
```

复杂度：**`O(1)`**。

---

#### 17.2.5 线性插值：`lerp`

```cpp
static double lerp(T a, T b, double t);
```

**语义：**

- 返回 `(1 - t) * a + t * b`；
- `t` 一般在 `[0,1]` 之间。

**用法：**

```cpp
double v0 = NumericUtils<double>::lerp(0.0, 10.0, 0.0);  // 0
double v5 = NumericUtils<double>::lerp(0.0, 10.0, 0.5);  // 5
double v1 = NumericUtils<double>::lerp(0.0, 10.0, 1.0);  // 10
```

复杂度：**`O(1)`**。

---

#### 17.2.6 对数收益率 / 简单收益率：`log_return` / `simple_return`

```cpp
template<typename U>
static double log_return(U current_price, U previous_price);

template<typename U>
static double simple_return(U current_price, U previous_price);
```

**语义：**

- `log_return`：
  - 若 `previous_price == 0`：返回 `0.0`；
  - 否则：返回 `log(current / previous)`。
- `simple_return`：
  - 若 `previous_price == 0`：返回 `0.0`；
  - 否则：返回 `(current - previous) / previous`。

**用法：**

```cpp
double p0 = 100.0;
double p1 = 102.0;

double r_log = NumericUtils<double>::log_return(p1, p0);    // ≈ 0.0198
double r_smp = NumericUtils<double>::simple_return(p1, p0); // 0.02
```

**注意事项：**

- `U` 必须是算术类型（int/float/double 等）；
- 适合直接在高频价序列上做逐 tick 收益计算；
- 复杂度都是 **`O(1)`**。

---

### 17.3 线性代数工具：`math/linear_algebra.h`

头文件：

```cpp
#include "math/linear_algebra.h"
using factorlib::math::LinearAlgebra;
```

这是一个以 `Eigen` 为后端的工具模板类，所有函数都是 `static`。

---

#### 17.3.1 协方差矩阵：`covariance_matrix`

**函数模板：**

```cpp
template<typename T>
class LinearAlgebra {
public:
    template<typename OuterContainer>
    static Eigen::MatrixXd covariance_matrix(const OuterContainer& data);
    // ...
};
```

- `OuterContainer`：外层容器，存一批样本；
- `OuterContainer::value_type`：需要是“可遍历的内层容器”（可以是 `std::vector<T>`、`std::array<T,N>` 等），每个内层代表一个样本向量。

**用法示例：**

```cpp
std::vector<std::vector<double>> samples = {
    {1.0, 2.0},
    {2.0, 1.0},
    {3.0, 0.0},
};

Eigen::MatrixXd cov =
    LinearAlgebra<double>::covariance_matrix(samples);
// cov 是 2×2 协方差矩阵
```

**实现要点与复杂度：**

1. 把 `samples` 拷贝到一个 `n × m` 的 Eigen 矩阵 `mat`：
  - `n = data.size()` 样本数；
  - `m = data.begin()->size()` 维度；
  - 拷贝复杂度：`O(n·m)`。
2. 计算列均值：`mean = mat.colwise().mean()`：`O(n·m)`。
3. 中心化：`centered = mat.rowwise() - mean.transpose()`。
4. 协方差矩阵：
   $$
   \Sigma = \frac{1}{n-1} \text{centered}^\top \cdot \text{centered}
   $$
   复杂度 `O(n·m²)`。

整体复杂度：**`O(n·m²)`**，适用于 `m` 不太大的情形（几十维以内）。

---

#### 17.3.2 条件数：`condition_number`

```cpp
static double condition_number(const Eigen::MatrixXd& matrix);
```

**语义：**

- 用奇异值分解（SVD）计算矩阵的 2 范数条件数：
  \[
  \kappa(A) = \frac{\sigma_{\max}}{\sigma_{\min}}
  \]
- 若矩阵为空，返回 `0.0`；
- 若最小奇异值为 0，返回 `+∞`。

**用法：**

```cpp
Eigen::MatrixXd A(2,2);
A << 1.0, 0.99,
     0.99, 0.98;

double kappa = LinearAlgebra<double>::condition_number(A);
// 值很大，表示矩阵病态
```

**复杂度：**

- 主要开销在 `Eigen::JacobiSVD` 上：
  - `O(m³)` 级别（m 为矩阵维度）。

---

#### 17.3.3 协方差正则化：`regularize_covariance`

```cpp
static Eigen::MatrixXd regularize_covariance(const Eigen::MatrixXd& cov_matrix,
                                             double regularization = 1e-6);
```

**语义：**

- 返回 `cov_matrix + λ I`，其中 `λ = regularization`；
- 用于防止协方差矩阵奇异 / 条件数过大；
- 典型用于回归 / L2 正则等场景。

**用法：**

```cpp
Eigen::MatrixXd cov = /* 通过 covariance_matrix 得到 */;
double lambda = 1e-4;

Eigen::MatrixXd cov_reg =
    LinearAlgebra<double>::regularize_covariance(cov, lambda);
```

**复杂度：**

- 仅仅是和单位矩阵相加：
  - 生成 `I`：`O(m²)`；
  - 加法：`O(m²)`；
- 总体：**`O(m²)`**。

---

#### 17.3.4 多元正态条件期望：`conditional_expectation`

```cpp
static double conditional_expectation(const Eigen::VectorXd& mean,
                                      const Eigen::MatrixXd& covariance,
                                      const Eigen::VectorXd& condition_values,
                                      size_t target_index);
```

**语义：**

- 对一个多元正态向量 `X = (X₁, ..., X_d)`，给定前 `d-1` 维的取值，计算
  第 `target_index` 维的条件期望（当前实现中，实际上假定 target 是最后一维；参数保留了 index 以便后续扩展）。
- 公式（2 维示例）：
  \[
  \mu_{2|1} = \mu_2 + \Sigma_{21} \Sigma_{11}^{-1} (x_1 - \mu_1)
  \]

**当前实现的约束：**

- 要求：
  - `covariance` 是方阵，维度 `d×d`；
  - `mean.size() == d`；
  - `condition_values.size() == d - 1`；
- 实际上按实现逻辑：
  - 把前 `n_condition = condition_values.size()` 维当作“条件变量”；
  - 把第 `n_condition` 这一维当作“目标变量”（即 target 是“紧接在条件变量之后的那一维”）。

**用法示例（2 维情形）：**

```cpp
// X = (X1, X2) ~ N(mean, covariance)
// 条件：给定 X1 = x1，求 E[X2 | X1 = x1]

Eigen::VectorXd mean(2);
mean << 0.0, 0.0;

Eigen::MatrixXd cov(2,2);
cov << 1.0, 0.5,
       0.5, 2.0;

// condition_values 只放条件变量（这里就是 X1 的取值）
Eigen::VectorXd cond(1);
cond << 1.0;

double mu_2_given_1 =
    LinearAlgebra<double>::conditional_expectation(mean, cov, cond, /*target_index*/ 1);
```

**实现细节：**

- 将协方差矩阵分块：
  - Σ₁₁：前 `n_condition × n_condition` 子矩阵；
  - Σ₁₂：前 `n_condition` 行、最后一列；
  - Σ₂₂：最后一行最后一列（标量）。
- 将均值向量分块：
  - μ₁：前 `n_condition` 维；
  - μ₂：第 `n_condition` 维。
- 用公式：
  $$
  \mu_{2|1} = \mu_2 + \Sigma_{21}\Sigma_{11}^{-1}(x_1 - \mu_1)
  $$
  在实现中，通过 `sigma_11.ldlt().solve(sigma_12)` 完成求解。

**时间复杂度：**

- 主开销在解线性方程 `Σ₁₁ β = Σ₁₂` 上：
  - 对 `n_condition × n_condition` 矩阵做 LDLᵀ 分解 → 大约 `O(n_condition³)`；
- 其余向量运算为 `O(n_condition)`；
- 整体：**`O(n_condition³)`**，通常 `n_condition` 不会太大（几维到十几维）。

---

这一节补全后，你就可以把 `distributions / numeric_utils / linear_algebra` 当成：

- **分布工具箱**：算分位点 / CDF / F 检验；
- **数值安全工具**：对数、除法、插值、收益率；
- **线性代数基础**：协方差矩阵、条件数、正则、条件期望；

直接在因子计算或模型配置里调用，不需要自己从头写计算公式。
