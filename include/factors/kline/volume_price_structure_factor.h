#pragma once

#include "core/ifactor.h"
#include "core/types.h"

#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

/**
 * @brief 配置：量价协同结构因子。
 *
 * 覆盖文档中的两个因子：
 *   - 16. 收益对数成交量相关；
 *   - 20. 量价第一主成分收益载荷。
 *
 * 参数说明：
 *   - window_size : 统计窗口长度（以 K 线根数计），默认 40。
 */
struct VolumePriceStructureConfig {
    int window_size = 40;
};

/**
 * @brief 因子：量价协同结构（16 & 20）。
 *
 * 1. 收益对数成交量相关（16）
 *    - 使用对数收益 r_t 与对数成交量 log(V_t) 的皮尔逊相关系数；
 *    - 反映“量价同步性”，例如放量上涨、缩量下跌等。
 *
 * 2. 量价第一主成分收益载荷（20）
 *    - 在三维变量 [r_t, log(V_t), 振幅_t] 上做协方差 PCA；
 *    - 找到解释方差最大的主成分 PC1，其特征向量记为 w = (w_r, w_v, w_amp)；
 *    - 因子取 w_r，即收益在第一主成分上的载荷（贡献度）。
 *
 * 计算特点：
 *   - 自身维护滑窗 deque，不依赖外部 RollWin 框架；
 *   - 每来一根 K 线，只更新窗口末端并丢弃最旧样本；
 *   - 相关系数与 PCA 都在当前窗口上重新计算，窗口较短时可接受。
 */
class VolumePriceStructureFactor : public BaseFactor {
public:
    using Code = std::string;

    /**
     * @brief 构造函数。
     *
     * 参数：
     *   - codes : 需要计算该因子的代码列表，如果为空则对所有代码计算；
     *   - cfg   : 配置（窗口长度等）。
     */
    VolumePriceStructureFactor(
        const std::vector<Code>& codes,
        const VolumePriceStructureConfig& cfg = VolumePriceStructureConfig{}
    );

    /**
     * @brief 注册本因子用到的 DataBus topic。
     *
     * topic 列表：
     *   - "kline/ret_logvol_corr" : 收益与 log 成交量相关（double）；
     *   - "kline/pv_pca1_ret_load": 量价三维 PCA 第一主成分中，收益维度的载荷。
     */
    static void register_topics(std::size_t capacity);

    /**
     * @brief 使用 K 线驱动因子更新。
     *
     * 流程：
     *   1. 按 instrument_id 找到 / 创建 CodeState；
     *   2. 调用 CodeState::push_bar 更新滑窗；
     *   3. 在窗口就绪时计算相关系数与 PCA，并发布因子值。
     */
    void on_bar(const Bar& b) override;

    /// 对该因子而言，多数情况下无需额外 flush，直接返回 true。
    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    /**
     * @brief 单只标的在本因子下的滑窗状态。
     *
     * 成员：
     *   - window      : 滑窗长度；
     *   - has_last_close / last_close : 用于构造对数收益；
     *   - returns     : 对数收益序列 r_t；
     *   - log_volumes : log 成交量 log(V_t)；
     *   - amplitudes  : 日内振幅 (high - low) / low。
     */
    struct CodeState {
        explicit CodeState(int window_size);

        int window;
        bool has_last_close;
        double last_close;
        struct Sample {
            double ret;
            double log_volume;
            double amplitude;
        };
        std::deque<Sample> samples;
        double sum_ret;
        double sum_log_volume;
        double sum_amp;
        double sum_ret2;
        double sum_log_volume2;
        double sum_amp2;
        double sum_ret_log_volume;
        double sum_ret_amp;
        double sum_log_volume_amp;

        /**
         * @brief 推进一根 K 线，内部自动计算对数收益 / log 成交量 / 振幅。
         */
        bool push_bar(const Bar& b);

        /**
         * @brief 当前滑窗是否已经填满，可以输出因子。
         */
        bool ready() const { return static_cast<int>(samples.size()) >= window; }
        std::size_t sample_count() const { return samples.size(); }

        /**
         * @brief 当前窗口内收益与对数成交量的皮尔逊相关。
         */
        double ret_logvol_corr() const;

        /**
         * @brief 计算 3x3 协方差矩阵。
         *
         * @param cov 输出数组，按行主序填充 3x3 协方差。
         * @return 若窗口长度 < 2 则返回 false。
         */
        bool covariance(double cov[3][3]) const;

    private:
        void add_sample(const Sample& sample);
        void remove_sample(const Sample& sample);
    };

    VolumePriceStructureConfig _cfg;               ///< 全局配置
    std::unordered_set<Code> _codes_filter;        ///< 代码白名单（为空表示全部）
    std::unordered_map<Code, CodeState> _states;   ///< 每个代码对应的滑窗状态

    /**
     * @brief 判断是否需要为指定代码计算本因子。
     */
    bool accept_code(const Code& code) const;
};

} // namespace factorlib
