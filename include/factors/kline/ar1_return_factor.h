#pragma once

#include "core/ifactor.h"
#include "core/types.h"
#include "math/rolling_ar.h"

#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

/**
 * @brief 配置：AR1 收益自回归系数因子（56）。
 *
 * 主要参数：
 *   - window_size : 滚动窗口长度 N，用于在 N 根 K 线上估计 AR(1) 系数。
 *
 * 一般推荐：
 *   - N = 40（文档中的默认设置，可在 20–60 之间调节）。
 */
struct Ar1ReturnConfig {
    int window_size = 40;
};

/**
 * @brief 因子：AR1 收益自回归系数。
 *
 * 因子含义：
 *   - 使用收盘价构造对数收益率序列 r_t = log(C_t / C_{t-1})；
 *   - 在长度为 N 的滑动窗口内，对 {r_t} 拟合 AR(1) 模型：
 *
 *       r_t = alpha + phi * r_{t-1} + epsilon_t
 *
 *   - 因子值取当前窗口估计得到的 phi，反映收益的“记忆性 / 惯性 / 均值回复”强弱。
 *
 * 计算特性：
 *   - 使用 math::RollingAR1 在滑窗内增量维护一阶 / 二阶矩；
 *   - 每来一根新 K 线，仅做 O(1) 次加减乘除，适合高频实时更新。
 */
class Ar1ReturnFactor : public BaseFactor {
public:
    using Code = std::string;

    /**
     * @brief 构造函数。
     *
     * 参数：
     *   - codes : 需要计算该因子的证券代码列表；
     *   - cfg   : 因子配置（窗口长度等）。
     *
     * 说明：
     *   - 若 codes 为空，则对所有代码生效；
     *   - 否则仅对 codes 中出现的代码计算因子。
     */
    Ar1ReturnFactor(
        const std::vector<Code>& codes,
        const Ar1ReturnConfig& cfg = Ar1ReturnConfig{}
    );

    /**
     * @brief 在 DataBus 上注册本因子所需的 topic。
     *
     * topic：
     *   - "kline/ar1_return_coeff"：AR1 收益自回归系数（double）。
     */
    static void register_topics(std::size_t capacity);

    /**
     * @brief 使用单根 K 线驱动因子更新。
     *
     * 调用流程：
     *   - 根据 instrument_id 找到/创建对应 CodeState；
     *   - 调用 CodeState::push_bar 更新滑窗收益；
     *   - 当 CodeState::ready() 返回 true 时，基于 RollingAR1 拟合 phi；
     *   - 通过 DataBus 发布因子值。
     */
    void on_bar(const Bar& b) override;

    /**
     * @brief 对该因子而言，通常不需要在收盘后再做额外 flush，直接返回 true。
     */
    bool force_flush() override { return true; }

private:
    /**
     * @brief 单只标的在本因子下的状态。
     *
     * 保存内容：
     *   - 最近 window_size 根 K 线的对数收益序列；
     *   - 对应的 AR(1) 滑窗估计器 RollingAR1。
     */
    struct CodeState {
        explicit CodeState(int window_size);

        int window;              ///< 滚动窗口长度（便于与配置对齐）
        bool has_last_close;     ///< 是否已经记录“上一根 K 线收盘价”
        double last_close;       ///< 上一根 K 线的收盘价（用于构造对数收益）
        std::deque<double> returns;      ///< 最近窗口内的收益序列（主要用于调试/检查）
        math::RollingAR1<double> ar1;    ///< AR(1) 滑窗估计器

        /**
         * @brief 推进一根新 K 线，更新收益序列与 AR(1) 状态。
         *
         * 步骤：
         *   1. 若当前为首根 K 线，仅记录 close，不产生收益；
         *   2. 否则计算对数收益 r_t = log(C_t / C_{t-1})；
         *   3. 将 r_t 推入 deque，并在超长时弹出最旧元素；
         *   4. 同步调用 ar1.push(r_t) 完成 AR(1) 滑窗增量更新。
         */
        void push_bar(const Bar& b);

        /**
         * @brief 当前是否已经积累了足够长度的窗口，可以输出因子。
         *
         * 这里直接转发给 RollingAR1::ready()，保证语义一致。
         */
        bool ready() const { return ar1.ready(); }
    };

    Ar1ReturnConfig _cfg;                    ///< 全局配置
    std::unordered_set<Code> _codes_filter;  ///< 需要计算的代码白名单（为空则为“全部”）
    std::unordered_map<Code, CodeState> _states; ///< 各代码对应的状态表

    /**
     * @brief 判断是否需要为指定 code 计算本因子。
     */
    bool accept_code(const Code& code) const;
};

} // namespace factorlib
