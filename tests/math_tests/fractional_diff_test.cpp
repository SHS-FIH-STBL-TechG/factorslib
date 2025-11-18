#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "math/fractional_diff.h"

using factorlib::math::FractionalDifferentiatorGL;

// 测试用例 1：验证 GL 分数阶差分的权重与增量输出是否与一次性卷积结果一致
// 场景：alpha=0.7, L=4，输入 x = [1,2,3,4,5]，使用类内部暴露的权重 w_k 直接手算期望值。
TEST(FractionalDiffTest, GLWeightsAndValue) {
    double alpha = 0.7;
    std::size_t L = 4;
    FractionalDifferentiatorGL<double> fd(alpha, L);

    std::vector<double> x{1,2,3,4,5};
    for (double v : x) fd.push(v);

    ASSERT_TRUE(fd.ready());  // 说明已经有 L+1 个样本，可以严格计算分数阶差分

    const auto& w = fd.weights();
    long double expected = 0.0L;
    expected += w[0]*5.0L;
    expected += w[1]*4.0L;
    expected += w[2]*3.0L;
    expected += w[3]*2.0L;
    expected += w[4]*1.0L;

    double got = fd.value();
    EXPECT_NEAR(got, static_cast<double>(expected), 1e-12);
}

// 测试用例 2：验证在样本不足 L+1 时，ready()==false 且 value() 返回 NaN
// 场景：alpha=0.5, L=3，只推入 3 个点，检查就绪标志与返回值。
TEST(FractionalDiffTest, NotReadyAndReturnNaN) {
    FractionalDifferentiatorGL<double> fd(0.5, 3);
    std::vector<double> x{1.0, 2.0, 3.0};
    for (double v : x) {
        fd.push(v);
    }
    // 只推入 3 个样本，而 L+1 = 4，因此尚未 ready
    EXPECT_FALSE(fd.ready());
    double v = fd.value();
    EXPECT_TRUE(std::isnan(v));  // 约定：未 ready 时返回 NaN
}

// 测试用例 3：验证 α=0 时退化为“恒等映射”：D^0 x_t = x_t
// 场景：alpha=0.0, L=3，输入 [1,2,3,4]，期望 value()==最后一个样本 4.0
TEST(FractionalDiffTest, ZeroOrderIsIdentity) {
    FractionalDifferentiatorGL<double> fd(0.0, 3);
    std::vector<double> x{1.0, 2.0, 3.0, 4.0};
    for (double v : x) fd.push(v);

    ASSERT_TRUE(fd.ready());
    double got = fd.value();
    // α=0 时，GL 权重满足 w_0=1, 其余 w_k≈0，因此分数阶差分应退化为当前值
    EXPECT_NEAR(got, 4.0, 1e-12);
}
