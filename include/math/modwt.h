#pragma once
/**
 * @file modwt.h
 * @brief Maximal Overlap DWT (MODWT) 增量实现（未降采样的小波变换）。
 *
 * 目标：支持小波多分辨分解的“能量占比”类因子（#3, #22）。
 * 做法：对每个尺度 j 使用上采样的滤波器 h_j, g_j 做卷积；不降采样，便于滑窗增量更新。
 * 复杂度：O(L_filter * J) 每步，L_filter 为母小波滤波器长度。
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

struct WaveletFilter {
    std::string name;
    std::vector<double> h; // scaling (low-pass)
    std::vector<double> g; // wavelet (high-pass)
};

/** 仅内置 db4 与 sym4 的滤波系数；可按需扩展。 */
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
 * @tparam BadPolicy 坏值策略（默认不处理）
 * 备注：MODWT 无降采样，适合流式增量；我们对每个尺度 j 维护最近 W 个 detail/scaling 系数的能量和。
 */
template<typename T=float, typename BadPolicy=NoCheckBadValuePolicy>
class RollingMODWT {
public:
    RollingMODWT(std::size_t W, int J, const WaveletFilter& wf)
        : W_(W), J_(J), wf_(wf) {
        if (W_ < 2 || J_ <= 0) throw std::invalid_argument("RollingMODWT: W>=2, J>=1");
        buf_.assign(W_, std::numeric_limits<T>::quiet_NaN());
        // 为每个尺度准备能量窗口（保存最近 W 个 detail/scaling 系数的平方，用于能量累积）
        detail_energy_.assign(J_, 0.0);
        detail_sqbuf_.assign(J_, std::vector<double>(W_, 0.0));
        detail_head_.assign(J_, 0);
    }

    bool push(T x) {
        if constexpr (std::is_floating_point_v<T>) {
            if (!is_finite_numeric(x)) {
                if (!BadPolicy::handle(x, "RollingMODWT::push")) {
                    return false;
                }
            }
        }
        if (size_ < W_) {
            buf_[tail_] = x;
            tail_ = (tail_ + 1) % W_;
            ++size_;
            // 未满时也可计算，但能量意义较弱；这类情况下 energy_ratio 将返回 NaN
            update_filters_for_index(size_-1); // 计算当前 index 的 wavelet/scaling；并更新能量缓冲
            return true;
        }

        // 滑窗：覆盖最老值
        buf_[head_] = x;
        head_ = (head_ + 1) % W_;
        tail_ = (tail_ + 1) % W_;
        update_filters_for_index(W_-1); // 始终认为 tail_ 的前一个为最新 index
        return true;
    }

    bool ready() const { return size_ >= W_; }

    /** 返回“趋势能量占比”：sum_{j=j_trend..J} E_j / sum_{j=1..J} E_j */
    double trend_energy_ratio(int j_trend) const {
        if (!ready()) return std::numeric_limits<double>::quiet_NaN();
        if (j_trend < 1) j_trend = 1;
        if (j_trend > J_) j_trend = J_;
        double trend = 0.0, total = 0.0;
        for (int j = 1; j <= J_; ++j) {
            double Ej = detail_energy_[j-1];
            total += Ej;
            if (j >= j_trend) trend += Ej;
        }
        if (total <= 0) return std::numeric_limits<double>::quiet_NaN();
        return trend / total;
    }

private:
    // 计算最近一个位置的各尺度 detail 系数，并以 O(1) 方式更新能量环形缓冲
    void update_filters_for_index(std::size_t idx_from_head) {
        // idx_from_head: [0..W_-1] 相对 head 的偏移
        std::size_t t = (head_ + idx_from_head) % W_;

        // 对每个尺度 j 计算 wavelet detail 系数 w_{j,t}
        for (int j = 1; j <= J_; ++j) {
            double wjt = detail_coeff_at(j, t);
            // 环形能量缓冲更新
            std::size_t hj = detail_head_[j-1];
            double& slot = detail_sqbuf_[j-1][hj];
            detail_energy_[j-1] += (wjt*wjt - slot);
            slot = wjt*wjt;
            detail_head_[j-1] = (hj + 1) % W_;
        }
    }

    // 计算给定尺度 j、时刻 t 的 detail 系数（MODWT 卷积，带尺度上采样）
    double detail_coeff_at(int j, std::size_t t) const {
        // 上采样步长 = 2^{j-1}，滤波器长度 Lf = wf_.g.size()
        int step = 1 << (j-1);
        const auto& g = wf_.g;
        int Lf = static_cast<int>(g.size());
        long double acc = 0.0L;
        for (int m = 0; m < Lf; ++m) {
            std::size_t tau = (t + W_ - (m*step)%W_) % W_;
            T xt = buf_[tau];
            if constexpr (std::is_floating_point_v<T>) {
                if (!is_finite_numeric(xt)) continue; // 忽略坏值
            }
            acc += static_cast<long double>(g[m]) * static_cast<long double>(xt);
        }
        return static_cast<double>(acc);
    }

private:
    std::size_t W_;
    int J_;
    WaveletFilter wf_;
    std::vector<T> buf_;
    std::size_t head_{0}, tail_{0}, size_{0};

    // 每尺度能量的 O(1) 更新缓存
    std::vector<double> detail_energy_;
    std::vector<std::vector<double>> detail_sqbuf_;
    std::vector<std::size_t> detail_head_;
};

}} // namespace factorlib::math
