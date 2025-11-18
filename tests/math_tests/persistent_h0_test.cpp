#include <gtest/gtest.h>
#include <vector>
#include "math/persistent_homology_h0.h"

using namespace factorlib::math;

TEST(PersistentH0Test, MeanPersistence1D) {
    std::vector<double> x{0.0,1.0,1.2,3.1,10.0};
    double got = mean_persistence_h0_1d(x, 2.0);
    // 期望值（由 gap/2<=2.0 的均值计算）：(0.5 + 0.1 + 0.95)/3 = 0.516666...
    double expected = (0.5 + 0.1 + 0.95)/3.0;
    EXPECT_NEAR(got, expected, 1e-12);
}
