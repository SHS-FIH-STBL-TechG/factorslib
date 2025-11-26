#include "gtest/gtest.h"
#include "math/hawkes_intensity.h"
#include <cmath>
#include <vector>

using namespace factorlib::math;

/**
 * @class HawkesIntensityTest
 * @brief Hawkes 过程强度估计器测试套件
 *
 * 测试覆盖：
 * - 正常功能测试：事件序列的强度演化
 * - 边界情况测试：无事件、持续事件、参数极端值
 * - 数学性质验证：平稳性、衰减特性、激励效应
 * - 数值稳定性测试：长时间运行、极端参数
 */
class HawkesIntensityTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置典型的 Hawkes 过程参数
        // α < β 确保过程平稳
        typical_mu = 0.1;
        typical_alpha = 0.5;
        typical_beta = 1.0;
        typical_dt = 1.0;
    }

    void TearDown() override {
        // 清理资源（如有需要）
    }

    double typical_mu;
    double typical_alpha;
    double typical_beta;
    double typical_dt;
};

/**
 * @test 测试单个事件对强度的激励效应
 * @brief 单个事件应该立即提升强度，然后逐渐衰减
 *
 * 验证 Hawkes 过程的核心特性：自激励。
 * 一个事件的发生会立即增加强度 α，然后按指数衰减。
 */
TEST_F(HawkesIntensityTest, SingleEventExcitation) {
    HawkesIntensity hawkes(typical_mu, typical_alpha, typical_beta, typical_dt);

    double initial_intensity = hawkes.value();

    // 触发一个事件
    double after_event = hawkes.update(1.0);

    // 验证强度增加了 α
    EXPECT_NEAR(after_event - initial_intensity, typical_alpha, 1e-10);

    // 后续无事件，强度应该衰减
    double next_step = hawkes.update(0.0);
    EXPECT_LT(next_step, after_event);  // 强度下降
    EXPECT_GT(next_step, typical_mu);   // 但仍高于基础强度
}

/**
 * @test 测试连续事件的强度累积
 * @brief 连续发生事件时，强度应该累积增加
 *
 * 模拟交易爆发场景：连续多个时间步都有事件发生。
 * 强度应该逐步累积，体现自激励的累积效应。
 */
TEST_F(HawkesIntensityTest, MultipleEventsAccumulation) {
    double mu = 0.1, alpha = 0.3, beta = 1.0;
    HawkesIntensity hawkes(mu, alpha, beta);

    std::vector<double> intensities;

    // 连续5个时间步都有事件
    for (int i = 0; i < 5; ++i) {
        intensities.push_back(hawkes.update(1.0));
    }

    // 验证强度单调递增（在 α < β 条件下）
    for (size_t i = 1; i < intensities.size(); ++i) {
        EXPECT_GT(intensities[i], intensities[i-1]);
    }

    // 验证强度高于基础强度
    EXPECT_GT(hawkes.value(), mu);
}

/**
 * @test 测试不同时间步长的影响
 * @brief 时间步长 Δt 影响衰减速度
 *
 * 较大的 Δt 导致更快的衰减，较小 Δt 导致更慢的衰减。
 * 这对应不同频率的数据采样。
 */
TEST_F(HawkesIntensityTest, DifferentTimeSteps) {
    // 小时间步长 - 慢衰减
    HawkesIntensity slow_decay(0.1, 0.5, 1.0, 0.1);
    // 大时间步长 - 快衰减
    HawkesIntensity fast_decay(0.1, 0.5, 1.0, 2.0);

    // 都触发一个事件
    slow_decay.update(1.0);
    fast_decay.update(1.0);

    double slow_intensity = slow_decay.value();
    double fast_intensity = fast_decay.value();

    // 然后都无事件一步
    slow_intensity = slow_decay.update(0.0);
    fast_intensity = fast_decay.update(0.0);

    // 大时间步长的衰减更多
    EXPECT_LT(fast_intensity, slow_intensity);
}

/**
 * @test 测试数值稳定性
 * @brief 长时间运行和极端事件序列下的数值稳定性
 *
 * 验证在长时间运行、大量事件等情况下不会出现数值问题
 *（如NaN、Inf、溢出等）。
 */
TEST_F(HawkesIntensityTest, NumericalStability) {
    HawkesIntensity hawkes(0.01, 0.8, 1.0);  // α接近β，但仍在平稳范围内

    // 长时间运行测试
    for (int i = 0; i < 10000; ++i) {
        double intensity = hawkes.update(i % 2);  // 交替事件
        EXPECT_TRUE(std::isfinite(intensity));
        EXPECT_GE(intensity, 0.0);  // 强度不应为负
    }

    // 最终强度应该在合理范围内
    double final_intensity = hawkes.value();
    EXPECT_TRUE(std::isfinite(final_intensity));
    EXPECT_GT(final_intensity, 0.0);
}

/**
 * @test 测试获取参数的方法
 * @brief 验证获取器方法返回正确的参数值
 */
TEST_F(HawkesIntensityTest, ParameterGetters) {
    double mu = 0.2, alpha = 0.6, beta = 1.2, dt = 0.5;
    HawkesIntensity hawkes(mu, alpha, beta, dt);

    EXPECT_DOUBLE_EQ(hawkes.mu(), mu);
    EXPECT_DOUBLE_EQ(hawkes.alpha(), alpha);
    EXPECT_DOUBLE_EQ(hawkes.beta(), beta);
    EXPECT_DOUBLE_EQ(hawkes.dt(), dt);
}

/**
 * @test 测试事件计数为小数的情况
 * @brief 支持连续值的事件计数（如成交量加权）
 *
 * 在某些应用场景中，n_t 可能是连续值，代表事件的"强度"或加权计数。
 */
TEST_F(HawkesIntensityTest, FractionalEventCount) {
    HawkesIntensity hawkes(0.1, 0.5, 1.0);

    // 使用小数事件计数（如部分成交、成交量加权）
    hawkes.update(0.5);  // 半个"事件"
    double intensity = hawkes.value();

    // 验证强度增加了 0.5 * α
    EXPECT_NEAR(intensity, 0.1 + 0.5 * 0.5, 1e-10);

    // 再更新一个完整事件
    intensity = hawkes.update(1.0);
    EXPECT_GT(intensity, 0.1 + 0.5 * 0.5);  // 强度进一步增加
}

/**
 * @test 测试无事件发生时的强度行为
 * @brief 当过程从稳态开始且没有事件时，强度保持不变
 *
 * 数学原理：如果 λ_t = μ 且 n_t = 0，则 λ_{t+1} = μ
 * 这体现了 Hawkes 过程的均值回归特性：从稳态开始就不会自发偏离。
 */
TEST_F(HawkesIntensityTest, NoChangeFromSteadyStateWithoutEvents) {
    HawkesIntensity hawkes(typical_mu, typical_alpha, typical_beta, typical_dt);

    const int steps = 10;
    std::vector<double> intensities;

    // 从稳态开始，连续多个时间步没有事件
    for (int i = 0; i < steps; ++i) {
        double intensity = hawkes.update(0.0);
        intensities.push_back(intensity);
    }

    // 验证强度保持为基础强度（没有事件就不会改变）
    for (size_t i = 0; i < intensities.size(); ++i) {
        EXPECT_NEAR(intensities[i], typical_mu, 1e-10);
    }
}

/**
 * @test 测试从非稳态开始的衰减行为
 * @brief 只有当强度偏离稳态时，无事件才会导致衰减
 */
TEST_F(HawkesIntensityTest, DecayFromNonSteadyState) {
    HawkesIntensity hawkes(typical_mu, typical_alpha, typical_beta, typical_dt);

    // 先触发事件使强度偏离稳态
    hawkes.update(1.0);
    double elevated = hawkes.value();
    EXPECT_GT(elevated, typical_mu);  // 确认强度提升

    // 然后观察衰减
    std::vector<double> intensities;
    for (int i = 0; i < 5; ++i) {
        intensities.push_back(hawkes.update(0.0));
    }

    // 验证强度递减并趋向稳态
    for (size_t i = 1; i < intensities.size(); ++i) {
        EXPECT_LT(intensities[i], intensities[i-1]);
    }
    EXPECT_NEAR(intensities.back(), typical_mu, 0.01);
}

/**
 * @test 测试离散时间 Hawkes 过程的平稳状态
 * @brief 验证离散时间版本的正确平稳状态强度
 *
 * 离散时间 Hawkes 过程在持续有事件时的平稳状态：
 * λ = μ + α/(1 - e^{-βΔt})
 * 这与连续时间版本不同，是离散化带来的特性。
 */
TEST_F(HawkesIntensityTest, DiscreteTimeStationaryState) {
    double mu = 0.1, alpha = 0.5, beta = 1.0, dt = 1.0;
    HawkesIntensity hawkes(mu, alpha, beta, dt);

    // 长时间持续事件
    for (int i = 0; i < 100; ++i) {
        hawkes.update(1.0);
    }

    double final_intensity = hawkes.value();

    // 计算离散时间理论平稳值
    double decay = std::exp(-beta * dt);
    double theoretical = mu + alpha / (1 - decay);

    EXPECT_NEAR(final_intensity, theoretical, 1e-10);
    EXPECT_NEAR(final_intensity, 0.890988, 1e-5);  // 精确验证
}

/**
 * @test 测试零基础强度下的衰减行为
 * @brief 当 μ=0 时，强度会指数衰减但需要足够步骤才能接近零
 *
 * 数学原理：λ_{t+1} = e^{-β} λ_t + α n_t
 * 无事件时，强度按几何序列衰减：λ_t = λ_0 × (e^{-β})^t
 * 衰减到ε以内需要的步骤数：t > ln(λ_0/ε) / β
 */
TEST_F(HawkesIntensityTest, ZeroBaseIntensityDecay) {
    HawkesIntensity no_base(0.0, 0.5, 1.0);

    // 触发一个事件
    no_base.update(1.0);
    double initial = no_base.value();  // 应该是 0.5

    // 需要更多步骤才能衰减到接近0
    const int steps_needed = 10;  // e^{-10} ≈ 0.000045
    for (int i = 0; i < steps_needed; ++i) {
        no_base.update(0.0);
    }

    // 现在应该很接近0了
    EXPECT_NEAR(no_base.value(), 0.0, 0.001);

    // 验证衰减规律：应该符合指数衰减
    HawkesIntensity verify(0.0, 0.5, 1.0);
    verify.update(1.0);
    for (int i = 0; i < 3; ++i) {
        double expected = 0.5 * std::pow(std::exp(-1.0), i+1);
        verify.update(0.0);
        EXPECT_NEAR(verify.value(), expected, 1e-10);
    }
}