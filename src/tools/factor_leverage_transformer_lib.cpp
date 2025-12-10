#include "tools/factor_leverage_transformer.h"

#include "core/databus.h"
#include "utils/log.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace factorlib::tools {

// ---------------------------------------------------------------------
// 本文件提供 FactorLeverageTransformer 的具体实现：负责订阅因子输出、
// 做滑窗正态化，并将新的杠杆值发布回 DataBus。该实现被工具和单元测试复用。
// ---------------------------------------------------------------------
namespace {

// 去重并清洗 code 列表，避免重复订阅与空串
std::vector<std::string> dedup_codes(std::vector<std::string> codes) {
    codes.erase(std::remove_if(codes.begin(), codes.end(),
                               [](const std::string& c) { return c.empty(); }),
                codes.end());
    std::sort(codes.begin(), codes.end());
    codes.erase(std::unique(codes.begin(), codes.end()), codes.end());
    return codes;
}

} // namespace

FactorLeverageTransformer::FactorLeverageTransformer(std::vector<FactorLeverageSpec> specs,
                                                     std::vector<std::string> codes,
                                                     std::size_t output_capacity)
    : _specs(std::move(specs)),
      _codes(dedup_codes(std::move(codes))),
      _output_capacity(output_capacity ? output_capacity : 2048) {}

FactorLeverageTransformer::~FactorLeverageTransformer() {
    stop();
}

void FactorLeverageTransformer::start() {
    if (_running) {
        return;
    }
    if (_specs.empty()) {
        LOG_WARN("FactorLeverageTransformer 启动失败：spec 列表为空");
        return;
    }
    if (_codes.empty()) {
        LOG_WARN("FactorLeverageTransformer 启动失败：code 列表为空");
        return;
    }

    auto& bus = DataBus::instance();
    for (const auto& spec : _specs) {
        if (spec.output_topic.empty()) {
            LOG_WARN("FactorLeverageTransformer: 输出 topic 为空，忽略该 spec");
            continue;
        }
        bus.register_topic<double>(spec.output_topic, _output_capacity);
    }

    _stop_requested = false;
    _running = true;
    _worker = std::thread(&FactorLeverageTransformer::worker_loop, this);

    for (std::size_t spec_idx = 0; spec_idx < _specs.size(); ++spec_idx) {
        const auto& spec = _specs[spec_idx];
        if (spec.input_topic.empty() || spec.output_topic.empty()) {
            continue;
        }
        for (const auto& code : _codes) {
            // 订阅每个 code 的原始因子输出，回调仅负责投递到本地队列
            DataBus::instance().subscribe<double>(
                spec.input_topic,
                code,
                [this, spec_idx](const std::string& cb_code, int64_t ts, const double& value) {
                    this->enqueue_event(spec_idx, cb_code, ts, value);
                });
        }
    }
}

void FactorLeverageTransformer::stop() {
    if (!_running) {
        return;
    }
    {
        std::lock_guard<std::mutex> lk(_queue_mtx);
        _stop_requested = true;
    }
    _queue_cv.notify_all();
    if (_worker.joinable()) {
        _worker.join();
    }
    _running = false;
    _stop_requested = false;
}

std::optional<SlidingGaussianLeverage::DistributionSnapshot>
FactorLeverageTransformer::stats(const std::string& input_topic,
                                 const std::string& code) const {
    std::lock_guard<std::mutex> lk(_state_mtx);
    auto key = make_state_key(input_topic, code);
    auto it = _states.find(key);
    if (it == _states.end()) {
        return std::nullopt;
    }
    return it->second.leverage.stats();
}

void FactorLeverageTransformer::enqueue_event(std::size_t spec_index,
                                              const std::string& code,
                                              int64_t ts_ms,
                                              double value) {
    if (!_running) {
        return;
    }
    Event evt;
    evt.spec_index = spec_index;
    evt.code = code;
    evt.ts_ms = ts_ms;
    evt.value = value;
    {
        std::lock_guard<std::mutex> lk(_queue_mtx);
        _queue.emplace_back(std::move(evt));
    }
    _queue_cv.notify_one();
}

void FactorLeverageTransformer::worker_loop() {
    // 单线程顺序消费事件队列，防止 DataBus 回调嵌套
    while (true) {
        Event evt;
        {
            std::unique_lock<std::mutex> lk(_queue_mtx);
            _queue_cv.wait(lk, [this] {
                return _stop_requested || !_queue.empty();
            });
            if (_stop_requested && _queue.empty()) {
                break;
            }
            evt = std::move(_queue.front());
            _queue.pop_front();
        }
        process_event(evt);
    }
}

void FactorLeverageTransformer::process_event(const Event& evt) {
    if (evt.spec_index >= _specs.size()) {
        return;
    }
    const auto& spec = _specs[evt.spec_index];
    if (!std::isfinite(evt.value) || spec.output_topic.empty()) {
        return;
    }

    double leverage_value = 0.0;
    {
        std::lock_guard<std::mutex> lk(_state_mtx);
        auto key = make_state_key(spec.input_topic, evt.code);
        auto [it, inserted] = _states.try_emplace(key, spec.window);
        auto leverage = it->second.leverage.transform(evt.value);
        if (!leverage.has_value()) {
            return;
        }
        leverage_value = *leverage;
    }

    safe_publish<double>(spec.output_topic, evt.code, evt.ts_ms, leverage_value);
}

} // namespace factorlib::tools
