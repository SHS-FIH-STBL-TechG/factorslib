#pragma once
/**
 * @file sliding_normal_eq.h
 * @brief 固定滑窗的增量法方程器：维护 X^T X 与 X^T y，支持 O( d^2 ) 增量更新与 O( d^3 ) 求解。
 *
 * 用法约定：
 *  - 维度 d 在构造时给定；窗口大小 N 在构造时给定。
 *  - 每次 push(x, y) 追加一条样本（x 为长度 d 的列向量；内部会做 x x^T 与 x*y 的增/减）。
 *  - 当样本数超过窗口 N 时，最旧样本会被自动移出（并从累计量中扣除）。
 *  - solve(beta, RSS) 使用 LDLT 求解最小二乘，返回是否成功；beta 为维度 d；RSS 为残差平方和。
 *
 * 数值说明：
 *  - XtX 可能在样本不足或高度共线时不满秩，此时 LDLT.info()!=Success 或 isNegative() 等会失败，返回 false。
 *  - 你在上层逻辑里已经做了“有效样本数/自由度”检查；这里保持简洁。
 */

#include <deque>
#include <utility>
#include <limits>
#include <cassert>
#include <Eigen/Dense>

namespace factorlib { namespace math {

template<typename T>
class SlidingNormalEq {
public:
    using Scalar = T;
    using Row    = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;  // 列向量 x (d x 1)

    SlidingNormalEq(int dim, int window)
        : d_(dim)
        , N_(window)
        , XtX_(dim, dim)
        , Xty_(dim)
        , beta_tmp_(dim)
    {
        assert(d_ > 0 && N_ > 0);
        XtX_.setZero();
        Xty_.setZero();
        // 注意：std::deque 没有 reserve，这里不再调用 reserve()
        samples_.clear();
        current_n_ = 0;
    }

    /// 追加一条样本；若超过窗口，则自动弹出最旧样本并从累计量中扣除
    void push(const Row& x, Scalar y) {
        assert(x.size() == d_);

        // 增量：累加 x x^T 与 x y
        XtX_.noalias() += (x * x.transpose());
        Xty_.noalias() += (x * y);
        samples_.emplace_back(x, y);
        ++current_n_;

        // 窗口裁剪
        if (current_n_ > N_) {
            const auto& old = samples_.front();
            const Row& xo   = old.first;
            const Scalar yo = old.second;
            XtX_.noalias() -= (xo * xo.transpose());
            Xty_.noalias() -= (xo * yo);
            samples_.pop_front();
            --current_n_;
        }
    }

    /// 解最小二乘（法方程）：返回是否成功；输出 beta 与 RSS
    bool solve(Row& beta_out, Scalar& RSS_out) const {
        if (current_n_ <= 0) return false;

        // LDLT 分解（对称）; 对奇异/不定情形做基本检查
        Eigen::LDLT<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> ldlt(XtX_);
        if (ldlt.info() != Eigen::Success) return false;
        if (ldlt.isNegative()) return false;  // XtX 若不半正定，放弃

        // beta = (XtX)^{-1} Xty
        beta_out = ldlt.solve(Xty_);
        if (ldlt.info() != Eigen::Success || !beta_out.allFinite()) return false;

        // RSS = sum (y - x^T beta)^2
        // 这里为了避免额外 O(N d) 的重扫，采用：RSS = y^T y - beta^T X^T y
        // 但我们没有累计 y^T y；故退而求其次，做一次 O(N d) 的重扫（N <= window_size）
        Scalar rss = Scalar(0);
        for (const auto& s : samples_) {
            const Row& x = s.first;
            const Scalar y = s.second;
            const Scalar e = y - x.dot(beta_out);
            rss += e * e;
        }
        RSS_out = rss;
        return RSS_out >= Scalar(0) && std::isfinite(static_cast<double>(RSS_out));
    }

    int dim()      const { return d_; }
    int window()   const { return N_; }
    int size()     const { return current_n_; }

    /// 重置为全零状态（保留配置）
    void reset() {
        XtX_.setZero();
        Xty_.setZero();
        samples_.clear();
        current_n_ = 0;
    }

private:
    int d_{0};
    int N_{0};
    int current_n_{0};

    // 累计量
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> XtX_;  // d x d
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1>              Xty_;  // d x 1

    // 样本窗口（存最近 N 条 (x, y) 用于在求解 RSS 时重扫；以及在弹出时扣回）
    std::deque<std::pair<Row, Scalar>> samples_;

    // 临时缓存（避免频繁分配）；目前 solve 里直接用输出参数即可，这里保留以便扩展
    mutable Row beta_tmp_;
};

}} // namespace factorlib::math
