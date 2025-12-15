#pragma once

#include "core/databus.h"
#include "core/ifactor.h"
#include "core/types.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

struct KalmanForecastZConfig {
    int vol_window = 60;
    double cR = 0.10;
    double cB = 0.01;
    int horizon_k = 5;
};

/**
 * @brief 因子：基于 Local Linear Trend Kalman Filter 的 k 步前向“预测收益”z-score。
 *
 * - 观测：y_t = log(C_t)
 * - 观测噪声：R_t = cR * mean_{Nvol}(Parkinson_sigma2)
 * - 斜率过程噪声：q_slope = cB * mean_{Nvol}(Parkinson_sigma2)；q_level=0
 * - 因子输出：Fk = (E[y_{t+k}] - y_t) / sqrt(Var[y_{t+k}])
 *
 * 注意：
 * - 该因子不使用未来价格，不存在未来函数；
 * - 输出为“原始信号”，正态化/杠杆化由工具侧处理。
 */
class KalmanForecastZFactor : public BaseFactor {
public:
    using Code = std::string;

    KalmanForecastZFactor(const std::vector<Code>& codes,
                          const KalmanForecastZConfig& cfg = KalmanForecastZConfig{});

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;
    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    struct RollingMean {
        explicit RollingMean(std::size_t window);
        double push(double x);
        std::size_t count() const { return _count; }

    private:
        std::vector<double> _buf;
        std::size_t _head = 0;
        std::size_t _count = 0;
        double _sum = 0.0;
    };

    struct LltKalman {
        double level = 0.0;
        double slope = 0.0;
        double P00 = 1.0;
        double P01 = 0.0;
        double P11 = 1.0;
        bool inited = false;

        void init(double y0);
        void step(double y, double q_level, double q_slope, double R);
        double forecast_return_z(int k, double y_t, double q_level, double q_slope, double R) const;
    };

    struct CodeState {
        explicit CodeState(std::size_t vol_window);
        RollingMean vol_mean;
        LltKalman kf;
    };

    static double safe_log(double x);
    static double parkinson_sigma2(const Bar& b);

    bool accept_code(const Code& code) const;

    KalmanForecastZConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;
};

} // namespace factorlib

