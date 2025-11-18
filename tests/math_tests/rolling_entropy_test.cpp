#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "math/rolling_entropy.h"

using factorlib::math::RollingHistogramEntropy;

// 测试目的：验证滑窗直方图熵的 O(1) 计数更新正确，
//           与窗口内“朴素重数统计 + 熵公式”的结果一致
static double naive_entropy(const std::vector<double>& win, const std::vector<double>& edges) {
    std::vector<int> cnt(edges.size()-1, 0);
    for (double x : win) {
        auto it = std::upper_bound(edges.begin(), edges.end(), x);
        std::size_t idx = (it==edges.begin()) ? 0 : (it - edges.begin() - 1);
        if (idx >= cnt.size()) idx = cnt.size()-1;
        cnt[idx]++;
    }
    double n = (double)win.size();
    double H = 0.0;
    for (int c : cnt) if (c>0) {
        double p = c/n;
        H -= p*std::log(p);
    }
    return H;
}

TEST(RollingEntropyTest, SlideAndMatchNaive) {
    std::size_t W=5;
    std::vector<double> edges{0,2,4,6,8,10};
    RollingHistogramEntropy<double> ent(W, edges);
    std::vector<double> seq{1,2,3,4,5,6,7,8,9,10};
    std::vector<double> win;
    for (double x : seq) {
        ent.push(x);
        win.push_back(x);
        if (win.size()>W) win.erase(win.begin());
        if (ent.ready()) {
            double got = ent.entropy();
            double exp = naive_entropy(win, edges);
            EXPECT_NEAR(got, exp, 1e-12);
        }
    }
}
