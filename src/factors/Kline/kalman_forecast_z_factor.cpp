#include "factors/kline/kalman_forecast_z_factor.h"

#include <algorithm>
#include <cmath>

namespace factorlib {

constexpr const char* TOP_KALMAN_FORECAST_Z = "kline/kalman_forecast_z";

static inline double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

double KalmanForecastZFactor::safe_log(double x) {
    return (x > 0.0) ? std::log(x) : std::numeric_limits<double>::quiet_NaN();
}

double KalmanForecastZFactor::parkinson_sigma2(const Bar& b) {
    if (!(b.high > 0.0) || !(b.low > 0.0)) return std::numeric_limits<double>::quiet_NaN();
    const double x = std::log(static_cast<double>(b.high) / static_cast<double>(b.low));
    const double denom = 4.0 * std::log(2.0);
    return (denom > 0.0) ? (x * x / denom) : std::numeric_limits<double>::quiet_NaN();
}

// ================= RollingMean =================

KalmanForecastZFactor::RollingMean::RollingMean(std::size_t window)
    : _buf(std::max<std::size_t>(1, window), 0.0) {}

double KalmanForecastZFactor::RollingMean::push(double x) {
    if (!std::isfinite(x)) x = 0.0;
    if (_count < _buf.size()) {
        _buf[_head++] = x;
        _sum += x;
        _count++;
        if (_head == _buf.size()) _head = 0;
    } else {
        _sum -= _buf[_head];
        _buf[_head] = x;
        _sum += x;
        _head++;
        if (_head == _buf.size()) _head = 0;
    }
    return (_count > 0) ? (_sum / static_cast<double>(_count)) : 0.0;
}

// ================= LltKalman =================

void KalmanForecastZFactor::LltKalman::init(double y0) {
    level = y0;
    slope = 0.0;
    P00 = 1.0;
    P01 = 0.0;
    P11 = 1.0;
    inited = true;
}

void KalmanForecastZFactor::LltKalman::step(double y, double q_level, double q_slope, double R) {
    if (!inited) init(y);

    // Predict: x^- = F x, F=[[1,1],[0,1]]
    level = level + slope;

    // P^- = F P F^T + Q
    const double P00p = P00 + 2.0 * P01 + P11 + q_level;
    const double P01p = P01 + P11;
    const double P11p = P11 + q_slope;

    const double innov = y - level;
    const double S = P00p + R;
    if (!(S > 1e-18) || !std::isfinite(S)) {
        P00 = P00p;
        P01 = P01p;
        P11 = P11p;
        return;
    }
    const double K0 = P00p / S;
    const double K1 = P01p / S;

    // Update x
    level += K0 * innov;
    slope += K1 * innov;

    // Joseph stabilized covariance:
    // P = (I-KH)P^-(I-KH)^T + K R K^T, H=[1,0]
    const double I00 = 1.0 - K0;
    const double I10 = -K1;

    const double A00 = I00 * P00p;
    const double A01 = I00 * P01p;
    const double A10 = I10 * P00p + P01p;
    const double A11 = I10 * P01p + P11p;

    const double nP00 = A00 * I00 + (K0 * K0) * R;
    const double nP01 = A00 * I10 + A01 * 1.0 + (K0 * K1) * R;
    const double nP11 = A10 * I10 + A11 * 1.0 + (K1 * K1) * R;

    P00 = nP00;
    P01 = nP01;
    P11 = nP11;
}

double KalmanForecastZFactor::LltKalman::forecast_return_z(int k,
                                                           double y_t,
                                                           double q_level,
                                                           double q_slope,
                                                           double R) const {
    if (!inited || k <= 0) return 0.0;

    double l = level;
    double b = slope;
    double C00 = P00;
    double C01 = P01;
    double C11 = P11;

    for (int i = 0; i < k; ++i) {
        l = l + b;
        const double C00p = C00 + 2.0 * C01 + C11 + q_level;
        const double C01p = C01 + C11;
        const double C11p = C11 + q_slope;
        C00 = C00p;
        C01 = C01p;
        C11 = C11p;
    }

    const double yk_mean = l;
    const double yk_var = C00 + R;
    if (!(yk_var > 1e-18) || !std::isfinite(yk_var)) return 0.0;

    const double m = yk_mean - y_t;
    return m / std::sqrt(yk_var);
}

// ================= Factor =================

KalmanForecastZFactor::CodeState::CodeState(std::size_t vol_window)
    : vol_mean(vol_window) {}

KalmanForecastZFactor::KalmanForecastZFactor(const std::vector<Code>& codes,
                                             const KalmanForecastZConfig& cfg)
    : BaseFactor("KalmanForecastZFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

bool KalmanForecastZFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void KalmanForecastZFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_KALMAN_FORECAST_Z, capacity);
}

void KalmanForecastZFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    const int vol_window = std::max(1, _cfg.vol_window);
    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(static_cast<std::size_t>(vol_window))).first;
    }
    CodeState& st = it->second;

    const double y = safe_log(static_cast<double>(b.close));
    if (!std::isfinite(y)) return;

    const double sig2 = parkinson_sigma2(b);
    const double sig2m = st.vol_mean.push(sig2);
    if (!std::isfinite(sig2m)) return;

    const double Rt = _cfg.cR * sig2m;
    const double qB = _cfg.cB * sig2m;
    const double qL = 0.0;

    st.kf.step(y, qL, qB, Rt);

    if (static_cast<int>(st.vol_mean.count()) < vol_window) {
        return;
    }

    const int k = std::max(1, _cfg.horizon_k);
    double Fk = st.kf.forecast_return_z(k, y, qL, qB, Rt);
    if (!std::isfinite(Fk)) return;
    Fk = clamp(Fk, -50.0, 50.0);

    safe_publish<double>(TOP_KALMAN_FORECAST_Z, b.instrument_id, b.data_time_ms, Fk);
}

} // namespace factorlib

