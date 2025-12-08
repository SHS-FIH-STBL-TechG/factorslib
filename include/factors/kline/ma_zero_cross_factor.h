#pragma once

#include "core/ifactor.h"
#include "core/types.h"

#include "math/rolling_zero_cross.h"

#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

/**
 * @brief 配置：均线零穿越率因子（53）。
 *
 * 参数：
 *   - window_size : 计算均线及零穿越率所使用的窗口长度。
 */
struct MaZeroCrossConfig {
    int window_size = 40;
};

/**
 * @brief 因子：均线零穿越率（53）。
 *
 * 直观含义：
 *   - 在一段价格序列中，以窗口均值为“零基线”，统计价格曲线在该基线
 *     上下穿越的频率；
 *   - 可以理解为“价格围绕均线振荡的剧烈程度”，在某些分形模型下与其
 *     几何粗糙度、路径维度相关。
 *
 * 实现说明：
 *   - 本实现采用批量方式：
 *       1) 在窗口内计算平均价；
 *       2) 构造去均值序列 dev_t = close_t - mean；
 *       3) 对 dev_t 使用 ZeroCrossUtils 统计零穿越次数与比例。
 *   - 窗口管理采用简单的 deque，不做复杂增量优化，长度不大时即可满足性能。
 */
class MaZeroCrossFactor : public BaseFactor {
public:
    using Code = std::string;

    /**
     * @brief 构造函数。
     *
     * 参数：
     *   - codes : 需要计算该因子的代码列表（为空表示全部）；
     *   - cfg   : 因子配置（窗口长度等）。
     */
    MaZeroCrossFactor(
        const std::vector<Code>& codes,
        const MaZeroCrossConfig& cfg = MaZeroCrossConfig{}
    );

    /**
     * @brief 注册因子输出所需的 DataBus topic。
     *
     * topic：
     *   - "kline/ma_zero_cross_rate" : 均线零穿越率。
     */
    static void register_topics(std::size_t capacity);

    /**
     * @brief 使用单根 K 线驱动因子计算。
     *
     * 流程：
     *   1. 根据 instrument_id 获取 / 创建 CodeState；
     *   2. 调用 CodeState::push_bar 更新价格滑窗；
     *   3. 在窗口就绪时构造去均值序列并统计零穿越率。
     */
    void on_bar(const Bar& b) override;

    /// 本因子无额外 flush 需求，直接返回 true。
    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    /**
     * @brief 单个代码对应的滑窗状态（仅需保存收盘价）。
     */
    struct CodeState {
        explicit CodeState(int window_size);

        int window;
        std::deque<double> closes;

        /**
         * @brief 推进一根 K 线，更新收盘价滑窗。
         */
        void push_bar(const Bar& b);

        /**
         * @brief 当前窗口是否已填满。
         */
        bool ready() const { return static_cast<int>(closes.size()) >= window; }
    };

    MaZeroCrossConfig _cfg;                    ///< 因子配置
    std::unordered_set<Code> _codes_filter;    ///< 代码白名单
    std::unordered_map<Code, CodeState> _states; ///< 各代码的滑窗状态

    /**
     * @brief 判断是否需要为指定代码计算本因子。
     */
    bool accept_code(const Code& code) const;
};

} // namespace factorlib
