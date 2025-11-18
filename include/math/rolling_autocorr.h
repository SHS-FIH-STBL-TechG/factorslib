#pragma once
/**
 * @file rolling_autocorr.h
 * @brief 固定窗口、固定滞后 k 的自相关系数 r_k —— 成对中心化的 O(1) 增量实现。
 *
 * 是否增量：✅（每步仅更新一对“最老”/“最新”的充分统计量 Σx、Σy、Σx²、Σy²、Σxy）
 * 复杂度：push O(1)，value O(1)；内存 O(W)。
 *
 * 说明：对窗口内配对序列 X_t=x_t、Y_t=x_{t-k}（样本数 N=W-k），
 *      r_k = cov(X,Y) / sqrt(var(X)var(Y))，其中协方差/方差均以**成对均值**居中。
 */
#include <vector>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <limits>
#include <type_traits>
#include "math/bad_value_policy.h"

namespace factorlib { namespace math {

template<typename T, typename BadValuePolicy = NoCheckBadValuePolicy>
class RollingAutoCorr {
public:
    RollingAutoCorr(std::size_t W, std::size_t k): W_(W), k_(k), buf_(W, T{}){
        if (k_>=W_) throw std::invalid_argument("k must be < W");
    }

    bool push(T v){
        if (!BadValuePolicy::handle(v, "RollingAutoCorr::push")) return true;
        if (filled_ < W_) {
            buf_[tail_] = v;
            tail_ = (tail_ + 1) % W_;
            filled_++;
            if (filled_==W_) init_pairs_();
            return true;
        }
        // 1) 移除“最老一对”：(x_{head+k}, x_{head})
        std::size_t old_idx = head_;
        std::size_t old_k_idx = (head_ + k_) % W_;
        T x_old = buf_[old_k_idx];
        T y_old = buf_[old_idx];
        remove_pair_(x_old, y_old);

        // 2) 覆盖最老位置为新值，窗口整体右移一步
        buf_[old_idx] = v;
        head_ = (head_ + 1) % W_;
        tail_ = (tail_ + 1) % W_;

        // 3) 加入“最新一对”：(x_new, x_{new-k})
        std::size_t new_k_idx = ((tail_ + W_) - 1 - k_) % W_;
        T x_new = v;
        T y_new = buf_[new_k_idx];
        add_pair_(x_new, y_new);
        return true;
    }

    bool ready() const { return filled_==W_; }

    double value() const {
        if (!ready()) return std::numeric_limits<double>::quiet_NaN();
        const long double N = static_cast<long double>(W_ - k_);
        long double mx = sum_x_ / N, my = sum_y_ / N;
        long double vx = sum_xx_/N - mx*mx;
        long double vy = sum_yy_/N - my*my;
        long double cov= sum_xy_/N - mx*my;
        if (vx<=0 || vy<=0) return 0.0;
        return static_cast<double>( cov / std::sqrt(vx*vy) );
    }

private:
    void init_pairs_(){
        sum_x_=sum_y_=sum_xx_=sum_yy_=sum_xy_=0.0L;
        for (std::size_t off=k_; off<W_; ++off){
            std::size_t i = (head_ + off) % W_;
            std::size_t j = (head_ + off - k_) % W_;
            add_pair_(buf_[i], buf_[j]);
        }
    }
    inline void add_pair_(T x, T y){
        long double lx = static_cast<long double>(x);
        long double ly = static_cast<long double>(y);
        sum_x_ += lx; sum_y_ += ly;
        sum_xx_ += lx*lx; sum_yy_ += ly*ly;
        sum_xy_ += lx*ly;
    }
    inline void remove_pair_(T x, T y){
        long double lx = static_cast<long double>(x);
        long double ly = static_cast<long double>(y);
        sum_x_ -= lx; sum_y_ -= ly;
        sum_xx_ -= lx*lx; sum_yy_ -= ly*ly;
        sum_xy_ -= lx*ly;
    }

private:
    std::size_t W_{0}, k_{0};
    std::vector<T> buf_;
    std::size_t head_{0}, tail_{0}, filled_{0};
    long double sum_x_{0}, sum_y_{0}, sum_xx_{0}, sum_yy_{0}, sum_xy_{0};
};

}} // namespace factorlib::math
