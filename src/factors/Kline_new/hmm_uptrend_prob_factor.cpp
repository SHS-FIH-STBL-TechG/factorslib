#include "factors/Kline_new/hmm_uptrend_prob_factor.h"

#include <cmath>

namespace factorlib {

// topic 名称
constexpr const char* TOP_HMM_UPTREND_PROB = "kline_new/hmm_uptrend_prob";
constexpr const char* TOP_HMM_NEXT_DRIFT   = "kline_new/hmm_next_drift";

//====================== CodeState 实现 ======================//

HmmUptrendProbFactor::CodeState::CodeState(const HmmUptrendProbConfig& cfg)
    : has_last_close(false),
      last_close(0.0),
      hmm(static_cast<std::size_t>(cfg.window_size), cfg.to_math_params()) {}

void HmmUptrendProbFactor::CodeState::push_bar(const Bar& b) {
    if (!(b.close > 0.0)) {
        return;
    }

    const double close = static_cast<double>(b.close);
    if (!has_last_close) {
        last_close = close;
        has_last_close = true;
        return;
    }

    // 对数收益：r_t = log(C_t / C_{t-1})
    const double ret = std::log(close / last_close);
    last_close = close;

    // 增量更新 HMM 滤波器
    hmm.push(ret);
}

//====================== 因子主体实现 ======================//

HmmUptrendProbFactor::HmmUptrendProbFactor(
    const std::vector<Code>& codes,
    const HmmUptrendProbConfig& cfg)
    : BaseFactor("HmmUptrendProbFactor", codes),
      _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

bool HmmUptrendProbFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

void HmmUptrendProbFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_HMM_UPTREND_PROB, capacity);
    bus.register_topic<double>(TOP_HMM_NEXT_DRIFT, capacity);
}

void HmmUptrendProbFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg)).first;
    }
    CodeState& st = it->second;

    st.push_bar(b);
    if (!st.ready()) {
        return;
    }

    // 从增量 HMM 滤波器获取结果
    const double gamma = st.hmm.uptrend_prob();
    const double mu_next = st.hmm.next_drift_prediction();

    if (!std::isfinite(gamma) || !std::isfinite(mu_next)) {
        return;
    }

    // 发布因子值
    safe_publish<double>(TOP_HMM_UPTREND_PROB,
                         b.instrument_id,
                         b.data_time_ms,
                         gamma);

    safe_publish<double>(TOP_HMM_NEXT_DRIFT,
                         b.instrument_id,
                         b.data_time_ms,
                         mu_next);
}

} // namespace factorlib
