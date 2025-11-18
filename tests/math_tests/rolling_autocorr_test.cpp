#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "math/rolling_autocorr.h"

using factorlib::math::RollingAutoCorr;

// 测试目的：用滑动窗口对线性递增序列做固定滞后自相关，
//           逐步滑动时，Rolling 结果应与朴素成对中心化计算一致（线性序列的 ρ≈1）
static double naive_autocorr(const std::vector<double>& win, std::size_t k) {
    std::size_t W = win.size();
    std::size_t N = W - k;
    long double sx=0, sy=0, sxx=0, syy=0, sxy=0;
    for (std::size_t i = k; i < W; ++i) {
        double x = win[i];
        double y = win[i-k];
        sx += x; sy += y; sxx += x*x; syy += y*y; sxy += x*y;
    }
    long double mx = sx/N, my=sy/N;
    long double vx = sxx/N - mx*mx;
    long double vy = syy/N - my*my;
    long double cov = sxy/N - mx*my;
    if (vx<=0 || vy<=0) return 0.0;
    return static_cast<double>( cov / std::sqrt(vx*vy) );
}

TEST(RollingAutoCorrTest, SlideAndMatchNaive) {
    std::size_t W=6, k=1;
    RollingAutoCorr<double> ac(W,k);
    std::vector<double> seq{1,2,3,4,5,6,7,8,9};
    std::vector<double> win;
    for (double x : seq) {
        ac.push(x);
        win.push_back(x);
        if (win.size()>W) win.erase(win.begin());
        if (ac.ready()) {
            double got = ac.value();
            double exp = naive_autocorr(win, k);
            EXPECT_NEAR(got, exp, 1e-12);
        }
    }
}
