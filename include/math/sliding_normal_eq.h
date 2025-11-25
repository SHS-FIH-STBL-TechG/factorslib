#pragma once
/**
 * @file sliding_normal_eq.h
 * @brief 固定滑窗增量 OLS（稳健版）：
 *        仍然增量维护窗口样本（用于出窗扣回），但 *求解* 时一次性构造 X，用 QR 求解，稳定性大幅提升。
 *
 * 复杂度：
 *  - push：O(d^2) 做累计（XtX/Xty 仅作调试参考，不再用于最终解）
 *  - solve：构造 X O(N·d)，QR O(N·d^2)，对 N≤300、d≤10 足够快且稳健
 */

#include <deque>
#include <utility>
#include <limits>
#include <cassert>
#include <Eigen/Dense>

#include "utils/log.h"

namespace factorlib { namespace math {

template<typename T>
class SlidingNormalEq {
public:
    using Scalar = T;
    using Row    = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;  // 列向量 x (d x 1)

    SlidingNormalEq(int dim, int window)
        : d_(dim), N_(window),
          XtX_(dim, dim), Xty_(dim) {
        assert(d_ > 0 && N_ > 0);
        XtX_.setZero();
        Xty_.setZero();
        samples_.clear();
        current_n_ = 0;
    }

    /// 追加一条样本；若超过窗口则弹出最旧
    void push(const Row& x, Scalar y) {
        assert(x.size() == d_);

        // 统一检查：x 或 y 中包含 NaN/Inf，则丢弃并记日志
        if (!x.allFinite() || !std::isfinite(static_cast<double>(y))) {
            LOG_DEBUG("SlidingNormalEq::push: sample has NaN/Inf, drop");
            return;
        }

        // 累计仅作参考，不再用它来求解（稳定性更好）
        XtX_.noalias() += (x * x.transpose());
        Xty_.noalias() += (x * y);

        samples_.emplace_back(x, y);
        ++current_n_;
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

    /// 用列主元 Householder QR 求解 OLS；返回 beta 与 RSS
    bool solve(Row& beta_out, Scalar& RSS_out) const {
        if (current_n_ <= 0) return false;

        // 1) 构造 X (n x d) 与 y (n)
        const int n = current_n_;
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> X(n, d_);
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1>              y(n);
        int i = 0;
        for (const auto& s : samples_) {
            X.row(i) = s.first.transpose();
            y(i)     = s.second;
            ++i;
        }

        // 2) QR 求解（比 XtX 的 LDLT 稳健）
        Eigen::ColPivHouseholderQR<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> qr(X);
        if (qr.info() != Eigen::Success) return false;

        beta_out = qr.solve(y);
        if (!beta_out.allFinite()) return false;

        // 3) RSS
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> e = y - X * beta_out;
        RSS_out = e.squaredNorm();
        return std::isfinite(static_cast<double>(RSS_out)) && RSS_out >= Scalar(0);
    }

    int dim()    const { return d_; }
    int window() const { return N_; }
    int size()   const { return current_n_; }

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

    // 仅作调试参考
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> XtX_;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1>              Xty_;

    // 滑窗样本（保留最近 N 条 (x, y)）
    std::deque<std::pair<Row, Scalar>> samples_;
};

}} // namespace factorlib::math
