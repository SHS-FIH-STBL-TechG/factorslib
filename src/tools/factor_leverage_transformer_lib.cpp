#include "factor_leverage_transformer.h"

#include "core/databus.h"
#include "utils/log.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace factorlib::tools {

namespace {

/**
 * @brief 去重并排序股票代码列表
 * @param codes 股票代码列表（可能包含重复和空字符串）
 * @return 去重排序后的代码列表
 */
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
    if (_running.exchange(true, std::memory_order_acq_rel)) {
        return;  // 已经在运行，直接返回
    }
    if (_specs.empty()) {
        LOG_WARN("FactorLeverageTransformer 启动失败：spec 列表为空");
        _running.store(false, std::memory_order_release);
        return;
    }
    if (_codes.empty()) {
        LOG_WARN("FactorLeverageTransformer 启动失败：code 列表为空");
        _running.store(false, std::memory_order_release);
        return;
    }

    // 在 DataBus 上注册所有输出 topic
    auto& bus = DataBus::instance();
    for (const auto& spec : _specs) {
        if (spec.output_topic.empty()) {
            LOG_WARN("FactorLeverageTransformer: 输出 topic 为空，忽略该 spec");
            continue;
        }
        bus.register_topic<double>(spec.output_topic, _output_capacity);
    }

    // 启动工作线程
    _stop_requested.store(false, std::memory_order_release);
    _subscriptions.clear();
    _worker = std::thread(&FactorLeverageTransformer::worker_loop, this);

    // 订阅所有 (spec, code) 组合
    for (std::size_t spec_idx = 0; spec_idx < _specs.size(); ++spec_idx) {
        const auto& spec = _specs[spec_idx];
        if (spec.input_topic.empty() || spec.output_topic.empty()) {
            continue;
        }
        for (const auto& code : _codes) {
            auto h = DataBus::instance().subscribe_handle<double>(
                spec.input_topic,
                code,
                [this, spec_idx](const std::string& cb_code, int64_t ts, const double& value) {
                    this->enqueue_event(spec_idx, cb_code, ts, value);
                });
            if (h) {
                _subscriptions.emplace_back(std::move(h));
            }
        }
    }
}

void FactorLeverageTransformer::stop() {
    if (!_running.exchange(false, std::memory_order_acq_rel)) {
        return;  // 已经停止，直接返回
    }
    {
        std::lock_guard<std::mutex> lk(_queue_mtx);
        _stop_requested.store(true, std::memory_order_release);
    }

    // 先取消订阅，避免 stop 期间持续 enqueue 导致 worker 无法排空队列退出。
    auto& bus = DataBus::instance();
    std::vector<factorlib::DataBus::SubscriptionHandle> subs;
    subs.swap(_subscriptions);
    for (const auto& h : subs) {
        bus.unsubscribe(h);
    }

    // 唤醒工作线程并等待其退出
    _queue_cv.notify_all();
    if (_worker.joinable()) {
        _worker.join();
    }
    _stop_requested.store(false, std::memory_order_release);
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
    if (!_running.load(std::memory_order_acquire)) {
        return;  // 未运行，忽略事件
    }
    Event evt;
    evt.spec_index = spec_index;
    evt.code = code;
    evt.ts_ms = ts_ms;
    evt.value = value;
    {
        std::lock_guard<std::mutex> lk(_queue_mtx);
        if (_stop_requested.load(std::memory_order_acquire)) {
            return;  // 已请求停止，不再入队
        }
        _queue.emplace_back(std::move(evt));
    }
    _queue_cv.notify_one();  // 唤醒工作线程
}

void FactorLeverageTransformer::worker_loop() {
    while (true) {
        Event evt;
        {
            std::unique_lock<std::mutex> lk(_queue_mtx);
            // 等待队列非空或收到停止请求
            _queue_cv.wait(lk, [this] {
                return _stop_requested.load(std::memory_order_acquire) || !_queue.empty();
            });
            // 停止请求且队列为空，退出循环
            if (_stop_requested.load(std::memory_order_acquire) && _queue.empty()) {
                break;
            }
            evt = std::move(_queue.front());
            _queue.pop_front();
        }
        process_event(evt);  // 处理事件
    }
}

void FactorLeverageTransformer::process_event(const Event& evt) {
    if (evt.spec_index >= _specs.size()) {
        return;  // 无效的 spec 索引
    }
    const auto& spec = _specs[evt.spec_index];
    if (!std::isfinite(evt.value) || spec.output_topic.empty()) {
        return;  // 无效的因子值或输出 topic
    }

    // 计算杠杆值
    double leverage_value = 0.0;
    {
        std::lock_guard<std::mutex> lk(_state_mtx);
        auto key = make_state_key(spec.input_topic, evt.code);
        // 查找或创建对应的转换状态
        auto [it, inserted] = _states.try_emplace(key, spec.window);
        // 使用滑窗高斯化转换器计算 z-score
        auto leverage = it->second.leverage.transform(evt.value);
        if (!leverage.has_value()) {
            return;  // 转换失败（窗口未满或其他原因）
        }
        leverage_value = *leverage;
    }

    // 发布杠杆值到输出 topic
    safe_publish<double>(spec.output_topic, evt.code, evt.ts_ms, leverage_value);
}

} // namespace factorlib::tools
