#include "factors/kline/ma_zero_cross_factor.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "core/databus.h"
#include "utils/log.h"

namespace factorlib {

namespace {
constexpr const char* TOP_MA_ZERO_CROSS_RATE = "kline/ma_zero_cross_rate";   // 53
} // anonymous namespace

//====================== CodeState 实现 ======================//

MaZeroCrossFactor::CodeState::CodeState(int window_size)
    : window(window_size) {}

/**
 * @brief 推进一根 K 线，更新收盘价滑窗。
 */
void MaZeroCrossFactor::CodeState::push_bar(const Bar& b) {
    if (!(b.close > 0.0)) return;
    const double close = static_cast<double>(b.close);

    closes.push_back(close);
    if (static_cast<int>(closes.size()) > window) {
        closes.pop_front();
    }
}

//====================== 因子主体实现 ======================//

MaZeroCrossFactor::MaZeroCrossFactor(
    const std::vector<Code>& codes,
    const MaZeroCrossConfig& cfg)
    : BaseFactor("MaZeroCrossFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

/**
 * @brief 判断是否需要为指定代码计算本因子。
 */
bool MaZeroCrossFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

/**
 * @brief 注册因子输出的 DataBus topic。
 */
void MaZeroCrossFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_MA_ZERO_CROSS_RATE, capacity);
}

/**
 * @brief 使用单根 K 线驱动“均线零穿越率”因子计算。
 */
void MaZeroCrossFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    // 获取 / 创建当前代码的滑窗状态
    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg.window_size)).first;
    }
    CodeState& st = it->second;

    st.push_bar(b);
    if (!st.ready()) return;

    // 在窗口内计算均值
    const std::size_t n = st.closes.size();
    std::vector<double> dev(n);
    double mean = std::accumulate(st.closes.begin(), st.closes.end(), 0.0) / static_cast<double>(n);
    for (std::size_t i = 0; i < n; ++i) {
        dev[i] = st.closes[i] - mean;
    }

    // 使用 ZeroCrossUtils 统计去均值序列的零穿越率
    const double rate = math::ZeroCrossUtils<double>::rate(dev, 0.0);
    if (!std::isfinite(rate)) return;

    safe_publish<double>(TOP_MA_ZERO_CROSS_RATE,
                         b.instrument_id,
                         b.data_time_ms,
                         rate);
}

} // namespace factorlib
