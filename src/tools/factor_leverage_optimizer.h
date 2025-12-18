#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace factorlib::tools {

/**
 * @brief 网格搜索配置：定义 θ 范围、风险基准等。
 * 
 * 用于控制 FactorLeverageOptimizer 的优化行为。
 */
struct LeverageSearchConfig {
    double theta_min = 0.0;      ///< z-score 阈值搜索范围下限
    double theta_max = 2.5;      ///< z-score 阈值搜索范围上限
    double theta_step = 0.05;    ///< z-score 阈值搜索步长

    int D = 4000;                ///< 风险基准样本天数（0 表示自动使用样本长度）
    double max_leverage = 2.0;   ///< 全局杠杆上界（绝对值）

    double z_cap_quantile = 0.99;  ///< z-score 裁剪分位数（用于计算 z_cap）
    double z_cap_min = 1.5;        ///< z_cap 下限
    double z_cap_max = 6.0;        ///< z_cap 上限

    // 对称性判定阈值：越大越“宽松”（更多分布会被认为对称，允许双边交易）。
    // 经验上 0.2 在大多数“主体对称、尾部少量极端值”的场景下更符合直觉。
    double symmetry_score_threshold = 0.2;
    double discrete_unique_ratio_threshold = 0.15;     ///< 离散性判定：唯一值比例阈值
    double discrete_max_freq_ratio_threshold = 0.03;   ///< 离散性判定：最大频率阈值
};

/**
 * @brief 因子分布类型
 * 
 * 根据因子的统计特征分类，决定交易模式。
 */
enum class FactorDistributionKind {
    SymmetricContinuous,      ///< 对称连续分布（双边交易）
    Asymmetric,               ///< 非对称分布（单边交易）
    DiscreteOrNonContinuous,  ///< 离散或非连续分布（多模式尝试）
};

/**
 * @brief 交易模式
 * 
 * 决定如何根据 z-score 的正负值建立仓位。
 */
enum class TradeMode {
    BothSides,      ///< 双边交易：z > θ 做多，z < -θ 做空
    PositiveOnly,   ///< 仅正向：z > θ 交易，其余空仓
    NegativeOnly,   ///< 仅负向：z < -θ 交易，其余空仓
};

/**
 * @brief 因子分布特征分析结果
 */
struct FactorProfile {
    FactorDistributionKind kind = FactorDistributionKind::SymmetricContinuous;  ///< 分布类型
    double symmetry_score = 0.0;   ///< 对称性得分（0 表示完全对称）
    double unique_ratio = 1.0;     ///< 唯一值比例（越小越离散）
    double max_freq_ratio = 0.0;   ///< 最大频率比例（越大越离散）
    double median = 0.0;           ///< 中位数
};

/**
 * @brief 阈值搜索结果，包含最优参数和评估指标
 */
struct ThresholdSearchResult {
    bool ok = false;  ///< 是否找到有效结果

    int D_sample_days = 0;   ///< 样本天数（用于交易频率约束等）
    int T_trade_days = 0;    ///< 评估样本长度 T（点数/天数）
    double theta = 0.0;      ///< 最优 z-score 阈值
    // 优化目标：Final_simple（单利口径 + 风险对齐 + 回撤惩罚）
    // Final_simple（当前口径）：
    //   Final_simple = sqrt(T / sum_t L_t^2) * ((252/T) * sum_t (L_t r_t)) / sqrt((252/T) * sum_t DD_t^2)
    // 其中：
    // - r_t 为对数收益；
    // - L_t 为最终 leverage（由 b_raw 经过风险对齐/裁剪得到）；
    // - E_t = sum_{i=1..t} L_i r_i，DD_t = max_{0<=u<=t} E_u - E_t（绝对回撤）。
    double score = -1e100;
    double baseline_score = std::numeric_limits<double>::quiet_NaN();  ///< 基准得分（1x 满仓持有）
    double final_equity = 1.0;   ///< 单利累计收益曲线的终值：E_T = sum_t (L_t r_t)
    double dd_rms = 0.0;         ///< DD 的均方根：sqrt((1/T) * sum_t DD_t^2)
    double c_scale = 1.0;        ///< 风险对齐系数：sqrt(252 / sum_t L_t^2)
    double z_cap = 3.0;          ///< z-score 裁剪值

    TradeMode mode = TradeMode::BothSides;  ///< 交易模式
    int polarity = +1;                      ///< 极性（+1 或 -1）
    std::size_t trade_days = 0;             ///< 实际交易天数
    double theta_raw_low = std::numeric_limits<double>::quiet_NaN();   ///< 原始因子值下阈值
    double theta_raw_high = std::numeric_limits<double>::quiet_NaN();  ///< 原始因子值上阈值
    FactorProfile profile;  ///< 因子分布特征
};

/**
 * @brief 单个时间点的杠杆交易详情
 * 
 * 用于记录回测过程中每个时间点的因子、杠杆、收益等信息。
 */
struct LeveragePoint {
    int64_t ts_ms = 0;      ///< 时间戳（毫秒）
    double x_raw = 0.0;     ///< 原始因子值
    double z = 0.0;         ///< z-score
    double b_raw = 0.0;     ///< 原始信号（未缩放）
    double leverage = 0.0;  ///< 杠杆序列（当前口径下等于 b_raw，本工具不再额外做风险缩放）
    double ret = 0.0;       ///< 下一期收益率
    double equity = 1.0;    ///< 累计权益
    double drawdown = 0.0;  ///< 回撤比例
    bool active = false;    ///< 是否有持仓
};

/**
 * @brief 因子杠杆优化器
 * 
 * 功能：
 * 1. 分析因子分布特征（对称性、离散性）
 * 2. 将因子值转换为 z-score
 * 3. 网格搜索最优阈值 θ 和交易模式
 * 4. 构建完整的杠杆交易序列
 * 
 * 优化目标：最大化 Final_simple（单利口径 + 风险对齐 + 回撤惩罚）
 * 
 * 约束条件：
 * - 杠杆绝对值不超过 max_leverage
 * - 风险对齐：使用 sqrt(252 / sum(b^2)) 作为统一缩放因子（用于打分，不再用于生成 leverage 序列）
 * - 最小交易频率：年化不低于 50 天
 */
class FactorLeverageOptimizer {
public:
    /**
     * @brief 构造函数
     * @param cfg 搜索配置参数
     */
    explicit FactorLeverageOptimizer(LeverageSearchConfig cfg);

    /**
     * @brief 分析因子分布特征
     * @param x 因子原始值序列
     * @return 因子分布特征（对称性、离散性等）
     * 
     * 功能：
     * 1. 计算中位数并去中心化
     * 2. 计算对称性得分：|(q10+q90)/2 - q50| / (q90-q10)
     * 3. 计算离散性指标：唯一值比例、最大频率
     * 4. 根据阈值分类为 Symmetric/Asymmetric/Discrete
     */
    FactorProfile analyze_profile(const std::vector<double>& x) const;
    
    /**
     * @brief 将因子值排名正态化为 z-score
     * @param x_centered 去中心化后的因子值
     * @return z-score 序列
     * 
     * 算法：
     * 1. 对每个值计算排名 rank（处理tie）
     * 2. 计算分位数 u = (rank - 0.5) / n
     * 3. 使用正态分布逆CDF：z = Φ^(-1)(u)
     */
    std::vector<double> rank_normalize_to_z(const std::vector<double>& x_centered) const;
    
    /**
     * @brief 搜索最优阈值和交易模式
     * @param ts 时间戳序列
     * @param x_raw 原始因子值
     * @param z z-score 序列
     * @param next_ret 下一期收益率
     * @param full_next_ret 完整样本收益率（用于计算baseline）
     * @param D_sample_days 样本天数
     * @return 最优搜索结果
     * 
     * 搜索过程：
     * 1. 分析因子分布，确定候选交易模式
     * 2. 计算 baseline 得分（1x 满仓持有）
     * 3. 对每个 (theta, mode, polarity) 组合：
     *    - 计算原始信号 b_raw
     *    - 计算风险缩放 c_scale
     *    - 模拟交易，计算 score
     * 4. 返回 score 最大的组合
     */
    ThresholdSearchResult search_best_threshold(const std::vector<int64_t>& ts,
                                                const std::vector<double>& x_raw,
                                                const std::vector<double>& z,
                                                const std::vector<double>& next_ret,
                                                const std::vector<double>& full_next_ret,
                                                int D_sample_days) const;

    // 训练/验证切分版本：在 train 段分析分布/确定 z_cap 等参数，在 val 段计算 Final_simple 并选择最优 θ。
    // - train_size + val_size 必须等于序列长度
    // - 返回的 score 为验证集上的 Final_simple
    ThresholdSearchResult search_best_threshold_train_val(const std::vector<int64_t>& ts,
                                                          const std::vector<double>& x_raw,
                                                          const std::vector<double>& z,
                                                          const std::vector<double>& next_ret,
                                                          int train_size,
                                                          int val_size) const;
    
    /**
     * @brief 根据最优参数构建杠杆交易序列
     * @param ts 时间戳序列
     * @param x_raw 原始因子值
     * @param z z-score 序列
     * @param next_ret 下一期收益率
     * @param D_sample_days 样本天数
     * @param best 最优搜索结果
     * @return 每个时间点的详细交易信息
     * 
     * 功能：
     * 1. 根据 best.theta 和 best.mode 计算每个时点的 b_raw
     * 2. 应用 c_scale 缩放并裁剪得到最终杠杆
     * 3. 模拟交易，计算权益曲线和回撤
     */
    std::vector<LeveragePoint> build_leverage_series(const std::vector<int64_t>& ts,
                                                     const std::vector<double>& x_raw,
                                                     const std::vector<double>& z,
                                                     const std::vector<double>& next_ret,
                                                     int D_sample_days,
                                                     const ThresholdSearchResult& best) const;

private:
    LeverageSearchConfig _cfg;  ///< 配置参数

    /**
     * @brief 最终化杠杆值（应用最大杠杆裁剪）
     * @param leverage 原始杠杆值
     * @param raw_signal 原始信号（用于判断是否为 0）
     * @return 裁剪后的杠杆值
     */
    double finalize_leverage(double leverage, double raw_signal) const;
};

} // namespace factorlib::tools
