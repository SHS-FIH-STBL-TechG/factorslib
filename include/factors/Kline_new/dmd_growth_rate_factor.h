#pragma once
/**
 * @file dmd_growth_rate_factor.h
 * @brief DMD（动态模态分解）主模态增长率因子。
 *
 * 理论依据：
 *   - 把"嵌入后的价格状态"视为近似线性演化：z_{k+1} ≈ A z_k
 *   - 用 DMD 从数据中反推 A 的主导特征值 λ
 *   - 连续时间增长率：ω = log(λ) / Δt
 *   - Re(ω) > 0 表示主模态在增长（上行趋势的动力学证据）
 *
 * 使用 math::RollingDMD 进行计算。
 */

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/databus.h"
#include "core/ifactor.h"
#include "core/types.h"
#include "math/rolling_dmd.h"

namespace factorlib {

/**
 * @brief DMD 主模态增长率因子配置。
 */
struct DmdGrowthRateConfig {
    int window_size = 40;   ///< 滚动窗口长度
    int embed_dim   = 4;    ///< 嵌入维数 m
    int rank        = 2;    ///< SVD 截断秩 r
};

/**
 * @brief DMD 动态模态分解主模态增长率因子。
 *
 * 使用 math::RollingDMD 进行计算。
 */
class DmdGrowthRateFactor : public BaseFactor {
public:
    using Code = std::string;

    DmdGrowthRateFactor(
        const std::vector<Code>& codes,
        const DmdGrowthRateConfig& cfg = DmdGrowthRateConfig{}
    );

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;
    void on_quote(const QuoteDepth&) override {}
    void on_tick(const CombinedTick&) override {}

    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    struct CodeState {
        explicit CodeState(const DmdGrowthRateConfig& cfg);

        math::RollingDMD<double> dmd;

        void push_bar(const Bar& b);
        bool ready() const { return dmd.ready(); }
    };

    DmdGrowthRateConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
