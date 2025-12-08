#include "factors/kline/ar1_return_factor.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace factorlib {

namespace {
// topic 名称：56. AR1 收益自回归系数
constexpr const char* TOP_AR1_RETURN_COEFF = "kline/ar1_return_coeff";
} // anonymous namespace

//====================== CodeState 实现 ======================//

Ar1ReturnFactor::CodeState::CodeState(int window_size)
    : window(window_size),
      has_last_close(false),
      last_close(0.0),
      returns(),
      ar1(static_cast<std::size_t>(window_size)) {}

/**
 * @brief 推进一根新 K 线，更新滑窗收益与 AR(1) 统计量。
 */
void Ar1ReturnFactor::CodeState::push_bar(const Bar& b) {
    // 价格非法时直接忽略
    if (!(b.close > 0.0)) {
        return;
    }

    const double close = static_cast<double>(b.close);
    if (!has_last_close) {
        // 第一根 K 线：仅记录 close，不产生收益
        last_close = close;
        has_last_close = true;
        return;
    }

    // 对数收益：r_t = log(C_t / C_{t-1})
    const double ret = std::log(close / last_close);
    last_close = close;

    // 维护本地窗口，用于调试/观测（不直接参与 AR(1) 计算）
    returns.push_back(ret);
    if (static_cast<int>(returns.size()) > window) {
        returns.pop_front();
    }

    // 同步更新 AR(1) 滑窗状态（内部维护一阶 / 二阶矩）
    ar1.push(ret);
}

//====================== 因子主体实现 ======================//

Ar1ReturnFactor::Ar1ReturnFactor(
    const std::vector<Code>& codes,
    const Ar1ReturnConfig& cfg)
    : BaseFactor("Ar1ReturnFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

/**
 * @brief 判断是否需要为指定代码计算本因子。
 */
bool Ar1ReturnFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

/**
 * @brief 在 DataBus 上注册本因子输出的 topic。
 */
void Ar1ReturnFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_AR1_RETURN_COEFF, capacity);
}

/**
 * @brief 使用单根 K 线驱动因子计算。
 */
void Ar1ReturnFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    // 获取 / 创建该标的对应的状态
    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg.window_size)).first;
    }
    CodeState& st = it->second;

    // 更新滑窗收益与 AR(1) 统计量
    st.push_bar(b);
    if (!st.ready()) {
        // 窗口尚未填满，不输出因子
        return;
    }

    // 在当前窗口上估计 AR(1) 系数
    double phi    = 0.0;
    double sigma2 = 0.0;
    if (!st.ar1.fit(phi, sigma2)) {
        return;
    }
    if (!std::isfinite(phi)) {
        return;
    }

    // 通过 DataBus 发布因子值
    safe_publish<double>(TOP_AR1_RETURN_COEFF,
                         b.instrument_id,
                         b.data_time_ms,
                         phi);
}

} // namespace factorlib
