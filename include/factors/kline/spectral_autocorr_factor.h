#pragma once

#include "core/ifactor.h"
#include "core/types.h"

#include "math/spectral_features.h"
#include "math/acf_energy.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

/**
 * @brief 配置：频域 + 自相关联合因子。
 *
 * 对应文档中的多个因子：
 *   - 37. 低频能量占比；
 *   - 39. 频谱重心；
 *   - 40. 频谱斜率；
 *   - 43. 自相关低阶能量；
 *   - 44. 自相关高阶能量；
 *   - 45. 自相关峰数量。
 *
 * 参数说明：
 *   - window_size       : 频域 / 自相关分析窗口长度 N；
 *   - low_freq_bins     : 低频能量占比中纳入的频点个数 K（不含 DC）；
 *   - ac_low_max_lag    : 低阶自相关能量最大滞后阶数；
 *   - ac_high_min_lag   : 高阶自相关能量起始滞后阶数；
 *   - ac_high_max_lag   : 高阶自相关能量最大滞后阶数；
 *   - ac_peak_threshold : 自相关峰数量统计时的下界阈值。
 */
struct SpectralAutocorrConfig {
    int window_size = 64;      ///< 频域/自相关窗口长度
    int low_freq_bins = 4;     ///< 低频能量计入的前 K 个频点（不含 DC）
    int ac_low_max_lag = 4;    ///< 自相关低阶能量最大阶数
    int ac_high_min_lag = 5;   ///< 自相关高阶能量起始阶数
    int ac_high_max_lag = 20;  ///< 自相关高阶能量最大阶数
    double ac_peak_threshold = 0.1; ///< 自相关峰阈值
};

/**
 * @brief 因子：频域 + 自相关联合特征。
 *
 * 使用对数收益率序列 r_t，在长度为 N 的窗口内计算：
 *   - 频域特征：
 *       * 37. 低频能量占比
 *       * 39. 频谱重心
 *       * 40. 频谱斜率
 *   - 时间域自相关特征：
 *       * 43. 自相关低阶能量
 *       * 44. 自相关高阶能量
 *       * 45. 自相关峰数量
 *
 * 说明：
 *   - 本因子主要关注“收益的记忆性、周期性与频谱结构”；
 *   - 使用 math::RollingSpectralFeatures 与 math::RollingACFEnergy 做增量维护；
 *   - 每来一根 K 线仅做 O(window_size + lag) 次更新，满足实时性要求。
 */
class SpectralAutocorrFactor : public BaseFactor {
public:
    using Code = std::string;

    /**
     * @brief 构造函数。
     *
     * 参数：
     *   - codes : 需要计算的代码列表（为空表示全部）；
     *   - cfg   : 因子配置。
     */
    SpectralAutocorrFactor(
        const std::vector<Code>& codes,
        const SpectralAutocorrConfig& cfg = SpectralAutocorrConfig{}
    );

    /**
     * @brief 注册本因子相关的 DataBus topic。
     *
     * topic 示例：
     *   - "kline/ret_spec_lowfreq_ratio" : 37. 低频能量占比；
     *   - "kline/ret_spec_centroid"      : 39. 频谱重心；
     *   - "kline/ret_spec_slope"         : 40. 频谱斜率；
     *   - "kline/ret_ac_low_energy"      : 43. 自相关低阶能量；
     *   - "kline/ret_ac_high_energy"     : 44. 自相关高阶能量；
     *   - "kline/ret_ac_peak_count"      : 45. 自相关峰数量。
     */
    static void register_topics(std::size_t capacity);

    /**
     * @brief 使用单根 K 线驱动因子更新。
     *
     * 步骤：
     *   1. 根据 instrument_id 找到 / 新建 CodeState；
     *   2. 调用 CodeState::push_bar 更新对数收益滑窗；
     *   3. 当窗口就绪时，对当前窗口做频域 + 自相关分析，并发布因子值。
     */
    void on_bar(const Bar& b) override;

    /// 本因子无额外 flush 需求，直接返回 true。
    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    /**
     * @brief 单个代码对应的滑窗状态。
     *
     * 仅维护最近 window_size 根 K 线的对数收益序列。
     */
    struct CodeState {
        explicit CodeState(const SpectralAutocorrConfig& cfg);

        int window;
        bool has_last_close;
        double last_close;
        math::RollingSpectralFeatures<double> spectral;
        math::RollingACFEnergy<double> acf;

        /**
         * @brief 推进一根 K 线，更新滚动频域/自相关状态。
         *
         * @return 窗口是否准备好输出指标。
         */
        bool push_bar(const Bar& b);

        /**
         * @brief 当前窗口是否已就绪。
         */
        bool ready() const { return spectral.ready() && acf.ready(); }

        /**
         * @brief 基于 RollingACFEnergy 统计峰数量。
         */
        int peak_count(std::size_t max_lag, double threshold) const;
    };

    SpectralAutocorrConfig _cfg;              ///< 因子配置
    std::unordered_set<Code> _codes_filter;   ///< 代码白名单
    std::unordered_map<Code, CodeState> _states; ///< 存储各代码的滑窗状态

    /**
     * @brief 判断是否需要为指定代码计算本因子。
     */
    bool accept_code(const Code& code) const;
};

} // namespace factorlib
