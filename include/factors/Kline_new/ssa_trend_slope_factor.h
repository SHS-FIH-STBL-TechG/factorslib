#pragma once
/**
 * @file ssa_trend_slope_factor.h
 * @brief SSA（奇异谱分析）趋势子空间重构后的端点斜率因子。
 *
 * 理论依据：
 *   - 把时间序列嵌入到 Hankel 轨迹矩阵
 *   - 做 SVD 分解，选择前 r 个奇异分量（趋势子空间）重构
 *   - 再做对角平均（Hankelization）得到趋势序列
 *   - 最终趋势因子取端点的 OLS 斜率
 *
 * 计算流程：
 *   - 使用 math::RollingSSA 进行增量窗口管理
 *   - push 是 O(1) 的，compute 需要批量 SVD
 */

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/databus.h"
#include "core/ifactor.h"
#include "core/types.h"
#include "math/rolling_ssa.h"

namespace factorlib {

/**
 * @brief SSA 趋势斜率因子配置。
 */
struct SsaTrendSlopeConfig {
    int window_size  = 40;   ///< 滚动窗口长度
    int L            = 10;   ///< Hankel 矩阵行数（嵌入维数）
    int r            = 2;    ///< 趋势子空间维数
    int slope_window = 5;    ///< 端点斜率计算窗口
};

/**
 * @brief SSA 趋势子空间端点斜率因子。
 *
 * 使用 math::RollingSSA 进行计算。
 */
class SsaTrendSlopeFactor : public BaseFactor {
public:
    using Code = std::string;

    SsaTrendSlopeFactor(
        const std::vector<Code>& codes,
        const SsaTrendSlopeConfig& cfg = SsaTrendSlopeConfig{}
    );

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;
    void on_quote(const QuoteDepth&) override {}
    void on_tick(const CombinedTick&) override {}

    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    struct CodeState {
        explicit CodeState(const SsaTrendSlopeConfig& cfg);

        math::RollingSSA<double> ssa;

        void push_bar(const Bar& b);
        bool ready() const { return ssa.ready(); }
    };

    SsaTrendSlopeConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
