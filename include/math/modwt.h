#pragma once
/**
 * @file modwt.h
 * @brief Maximal Overlap DWT (MODWT) 增量实现（未降采样的小波变换）。
 *
 * 目标：支持小波多分辨分解的“能量占比”类因子（#3, #22）。
 * 做法：对每个尺度 j 使用上采样的滤波器 h_j, g_j 做卷积；不降采样，适合滑动窗口的增量更新。
 * 复杂度：O(L_filter * J) 每步，L_filter 为母小波滤波器长度，J 为分解层数。
 *
 * 这里实现的是“Rolling” 版本：
 *   - 维护固定长度为 W 的滑动窗口；
 *   - 每 push 一个新样本，就更新所有尺度的 detail 系数能量和；
 *   - 提供 trend_energy_ratio(j_trend) 计算“趋势能量占比”：
 *       ratio = ( ∑_{j >= j_trend} E_j ) / ( ∑_{j=1..J} E_j )
 */

#include <vector>
#include <deque>
#include <cmath>
#include <limits>
#include <string>
#include <stdexcept>

#include "math/bad_value_policy.h"

namespace factorlib {
namespace math {

/**
 * @brief 小波滤波器（母小波的缩放 / 小波滤波器）
 *
 * h: scaling filter（低通）
 * g: wavelet filter（高通）
 *
 * 注意：这里存的是“母小波”层面的滤波器系数；
 * 在 MODWT 中，各尺度 j 的滤波器由母小波滤波器“上采样 + 归一化”得到。
 */
struct WaveletFilter {
    std::string name;
    std::vector<double> h; // scaling (low-pass)
    std::vector<double> g; // wavelet (high-pass)
};

/**
 * @brief 内置 Daubechies 4 (db4) 小波滤波器
 *
 * 这里按常见定义给出 db4 的系数，并在末尾补零到长度 8，以便和 sym4 一致。
 * 只要 h/g 的能量接近 1，即可保证是正交小波。
 */
inline WaveletFilter wavelet_db4() {
    // Daubechies 4 (length 8) - 常见定义
    const double h[] = {
        0.4829629131445341,
        0.8365163037378079,
        0.2241438680420134,
        -0.12940952255126034,
        0.0,0.0,0.0,0.0 // pad到8，便于一致性
    };
    const double g[] = {
        -0.12940952255126034,
        -0.2241438680420134,
        0.8365163037378079,
        -0.4829629131445341,
        0.0,0.0,0.0,0.0
    };
    return WaveletFilter{"db4", std::vector<double>(h,h+8), std::vector<double>(g,g+8)};
}

/**
 * @brief 内置 Symlet 4 (sym4) 小波滤波器
 *
 * sym4 和 db4 类似，同为长度 8 的正交小波，时间对称性更好。
 */
inline WaveletFilter wavelet_sym4() {
    const double h[] = {
        -0.07576571478935668,
        -0.02963552764599851,
        0.49761866763201545,
        0.8037387518059161,
        0.29785779560527736,
        -0.09921954357684722,
        -0.012603967262037833,
        0.0322231006040427
    };
    const double g[] = {
        -0.0322231006040427,
        -0.012603967262037833,
        0.09921954357684722,
        0.29785779560527736,
        -0.8037387518059161,
        0.49761866763201545,
        0.02963552764599851,
        -0.07576571478935668
    };
    return WaveletFilter{"sym4", std::vector<double>(h,h+8), std::vector<double>(g,g+8)};
}

/**
 * @tparam T         输入数据类型（通常 double/float）
 * @tparam BadPolicy 坏值策略（来自 math/bad_value_policy.h，负责处理 NaN / inf 等）
 *
 * RollingMODWT 的核心特性：
 *   - 固定窗口长度 W；
 *   - 分解层数 J（尺度 j = 1..J）；
 *   - 每个尺度 j 维护一个“最近 W 个 detail 系数平方”的环形缓冲，用于能量和 E_j 的 O(1) 更新；
 *   - ready() 返回 true 时，表示窗口已填满（size_ >= W），此时 trend_energy_ratio 才有明确意义。
 */
template<typename T=float, typename BadPolicy=NoCheckBadValuePolicy>
class RollingMODWT {
public:
    /**
     * @brief 构造函数
     * @param W   滑动窗口长度（>=2）
     * @param J   小波分解层数（>=1）
     * @param wf  小波滤波器（db4 / sym4 等）
     *
     * 注意：
     *   - 这里只存一份母小波滤波器 wf_，各尺度的“上采样”逻辑在 detail_coeff_at 中处理；
     *   - 对每个尺度 j，我们都维护：
     *       - detail_energy_[j-1]：当前窗口内 detail 系数平方和 E_j；
     *       - detail_sqbuf_[j-1]：长度为 W 的环形数组，存 w_{j,t}^2；
     *       - detail_head_[j-1]：当前要覆盖的位置索引。
     */
    RollingMODWT(std::size_t W, int J, const WaveletFilter& wf)
        : W_(W), J_(J), wf_(wf) {
        if (W_ < 2 || J_ <= 0) {
            throw std::invalid_argument("RollingMODWT: W>=2, J>=1");
        }

        // 原始数据窗口，长度为 W；使用 [head_, tail_) 这种环形缓冲方式管理
        buf_.assign(W_, std::numeric_limits<T>::quiet_NaN());

        // 为每个尺度准备能量窗口：
        //   - detail_energy_[j] 记录该尺度 j+1 在整个窗口内的能量和 E_j；
        //   - detail_sqbuf_[j] 是长度 W 的 w_{j,t}^2 环形缓冲，用于 O(1) 更新 E_j；
        //   - detail_head_[j] 记录环形缓冲当前写入的位置。
        detail_energy_.assign(J_, 0.0);
        detail_sqbuf_.assign(J_, std::vector<double>(W_, 0.0));
        detail_head_.assign(J_, 0);
    }

    /**
     * @brief 向 MODWT 滑动窗口推入一个新样本
     *
     * 逻辑：
     *   1. 若 BadPolicy 认为 x 是坏值并且 handle 返回 false，则本次 push 直接失败；
     *   2. 窗口未填满时，把 x 放到 tail_ 位置，size_++；
     *   3. 窗口填满后，每 push 一个新点就覆盖最老的数据（head_ 前进，tail_ 前进）；
     *   4. 每次 push 之后，调用 update_filters_for_index(...)：
     *        - 计算“当前最新位置”的各尺度 detail 系数；
     *        - 用“新平方和 - 旧平方和”方式更新能量和。
     *
     * @return true  表示成功处理该样本
     * @return false 表示因为坏值策略拒绝该样本
     */
    bool push(T x) {
        if constexpr (std::is_floating_point_v<T>) {
            if (!is_finite_numeric(x)) {
                // 由 BadPolicy 决定如何处理 NaN / inf 等坏值
                if (!BadPolicy::handle(x, "RollingMODWT::push")) {
                    return false;
                }
            }
        }

        if (size_ < W_) {
            // 窗口尚未填满：只累积数据，head_ 固定为 0，tail_ 向前推进
            buf_[tail_] = x;
            tail_ = (tail_ + 1) % W_;
            ++size_;

            // 未满时也可以计算 detail 系数并累计能量，但这时“能量占比”的物理意义较弱，
            // 因此 trend_energy_ratio 会在 ready()==false 时直接返回 NaN。
            update_filters_for_index(size_ - 1); // 索引从 head_ 开始的偏移
            return true;
        }

        // 窗口已满：覆盖最老的样本（head_），head_ 与 tail_ 共同前进
        buf_[head_] = x;
        head_ = (head_ + 1) % W_;
        tail_ = (tail_ + 1) % W_;

        // 这里我们约定“最新”位置是与 head_ 间隔 W_-1 的位置
        update_filters_for_index(W_ - 1);
        return true;
    }

    /**
     * @brief 当前窗口是否已经“填满”
     *
     * 只有 ready()==true 时，才能认为窗口中有 W 个有效样本，
     * 此时 trend_energy_ratio 才有真正的统计意义。
     */
    bool ready() const { return size_ >= W_; }

    /**
     * @brief 计算“趋势能量占比”
     *
     * @param j_trend 视为“趋势”的起始尺度（1 <= j_trend <= J_）
     *        - 如果传入 <1，则自动提升为 1；
     *        - 如果传入 >J_，则自动收缩为 J_。
     *
     * 定义：
     *   ratio = ( ∑_{j=j_trend..J} E_j ) / ( ∑_{j=1..J} E_j )
     *
     * 数值处理细节：
     *   - 若 !ready()，直接返回 NaN（窗口未满，统计不可靠）；
     *   - 由于浮点数舍入误差，detail_energy_ 可能偶尔出现极小的负数（如 -1e-15），
     *     这在物理上是不可能的，因此在这里统一截断为 0；
     *   - 若总能量 total <= 0，则返回 NaN（说明信号几乎常数或能量极低）；
     *   - ratio 理论范围是 [0,1]，为了避免数值误差导致的 1.00000000002 等伪像，
     *     这里对结果再做一次 [0,1] 的钳制。
     */
    double trend_energy_ratio(int j_trend) const {
        if (!ready()) {
            // 窗口尚未填满：返回 NaN，让上层自行决策（可以忽略或用于调试）
            return std::numeric_limits<double>::quiet_NaN();
        }

        if (j_trend < 1) j_trend = 1;
        if (j_trend > J_) j_trend = J_;

        double trend = 0.0;
        double total = 0.0;

        for (int j = 1; j <= J_; ++j) {
            double Ej = detail_energy_[j - 1];

            // 数值稳定处理：能量物理上应当非负，
            // 若出现极小的负值（由浮点误差引起），统一修正为 0。
            if (Ej < 0.0) {
                Ej = 0.0;
            }

            total += Ej;
            if (j >= j_trend) {
                trend += Ej;
            }
        }

        if (total <= 0.0) {
            // 总能量为 0（或数值上接近 0），无法定义比例，返回 NaN。
            return std::numeric_limits<double>::quiet_NaN();
        }

        double ratio = trend / total;

        // 再做一次数值钳制，避免得到略微超出 [0,1] 的比例（比如 1.0000000001）
        if (ratio < 0.0) ratio = 0.0;
        if (ratio > 1.0) ratio = 1.0;

        return ratio;
    }

private:
    /**
     * @brief 在“从 head_ 起的 idx_from_head 位置”计算所有尺度的 detail 系数，
     *        并用新平方和更新每个尺度的能量环形缓冲。
     *
     * @param idx_from_head [0..W_-1]，表示距离 head_ 的偏移量：
     *        t = (head_ + idx_from_head) % W_ 即为真实的时间索引。
     *
     * 逻辑：
     *   - 对每个尺度 j：
     *       1. 通过 detail_coeff_at(j, t) 计算 w_{j,t}；
     *       2. 在该尺度的环形缓冲 detail_sqbuf_[j-1] 中找到当前写入位置 hj；
     *       3. detail_energy_[j-1] += (w_{j,t}^2 - old_slot)；
     *       4. 覆盖 old_slot，并推进环形指针 hj。
     *
     * 这样一来，每步更新能量的复杂度为 O(J)，而不是 O(W*J)。
     */
    void update_filters_for_index(std::size_t idx_from_head) {
        // idx_from_head: [0..W_-1] 相对 head_ 的偏移
        std::size_t t = (head_ + idx_from_head) % W_;

        // 对每个尺度 j 计算 wavelet detail 系数 w_{j,t}
        for (int j = 1; j <= J_; ++j) {
            double wjt = detail_coeff_at(j, t);

            // 环形能量缓冲更新：
            //   - detail_sqbuf_[j-1][hj] 是当前要被“覆盖”的旧平方和；
            //   - detail_energy_[j-1] 加上“新平方和 - 旧平方和”即可保持窗口内平方和。
            std::size_t hj = detail_head_[j - 1];
            double& slot = detail_sqbuf_[j - 1][hj];

            detail_energy_[j - 1] += (wjt * wjt - slot);
            slot = wjt * wjt;

            // 环形指针前进
            detail_head_[j - 1] = (hj + 1) % W_;
        }
    }

    /**
     * @brief 计算给定尺度 j、时刻 t 的 detail 系数 w_{j,t}
     *
     * MODWT 与普通 DWT 的差别：
     *   - 不做降采样（no decimation），所以每一个时间点都会有一套 {w_{j,t}}；
     *   - 每个尺度 j 的滤波器等价于“母滤波器 g 上采样 step = 2^{j-1}”，
     *     并做 1/sqrt(step) 的归一化，以保持不同尺度的能量可比。
     *
     * 周期边界条件：
     *   - MODWT 常见实现是“周期延拓”，即索引在窗口内按 mod W_ 取值。
     *
     * @param j 尺度索引（1 <= j <= J_）
     * @param t 环形缓冲中的时间索引（0 <= t < W_）
     */
    double detail_coeff_at(int j, std::size_t t) const {
        // 上采样步长 = 2^{j-1}，滤波器长度 Lf = wf_.g.size()
        int step = 1 << (j - 1);
        const auto& g = wf_.g;
        int Lf = static_cast<int>(g.size());

        // MODWT 理论中，不同尺度的滤波器需要乘以 1/sqrt(step) 做归一化，
        // 以保证“整体能量守恒”，防止高尺度能量被放大导致趋势能量比 > 1。
        long double scale = 1.0L / std::sqrt(static_cast<long double>(step));

        long double acc = 0.0L;
        for (int m = 0; m < Lf; ++m) {
            // 周期边界：t - m*step 可能为负或超出 W_，统一按 mod W_ 处理
            std::size_t tau = (t + W_ - (m * step) % W_) % W_;
            T xt = buf_[tau];

            if constexpr (std::is_floating_point_v<T>) {
                // 对于浮点数类型，遇到坏值则跳过该项贡献（等价于系数为 0）
                if (!is_finite_numeric(xt)) continue;
            }

            acc += static_cast<long double>(g[m]) * static_cast<long double>(xt);
        }

        // 应用尺度归一化
        return static_cast<double>(acc * scale);
    }

private:
    // ===================== 核心参数 =====================

    std::size_t W_;   ///< 滑动窗口长度
    int         J_;   ///< 小波分解层数
    WaveletFilter wf_;///< 母小波滤波器（db4 / sym4 等）

    // ===================== 原始数据环形缓冲 =====================

    std::vector<T> buf_;        ///< 长度为 W_ 的原始数据缓冲
    std::size_t    head_{0};    ///< 指向最老样本的位置
    std::size_t    tail_{0};    ///< 指向下一个要写入的位置
    std::size_t    size_{0};    ///< 当前已经填入的样本数量（<= W_）

    // ===================== 每个尺度的能量缓存 =====================

    std::vector<double>              detail_energy_; ///< E_j：当前窗口内第 j 尺度的 detail 系数能量和
    std::vector<std::vector<double>> detail_sqbuf_;  ///< w_{j,t}^2 的环形缓冲，尺寸 [J_][W_]
    std::vector<std::size_t>         detail_head_;   ///< 每个尺度对应的环形写入位置
};

}} // namespace factorlib::math
