#pragma once

#include "tools/sliding_gaussian_leverage.h"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace factorlib::tools {

/**
 * @brief 描述一条“因子 → 杠杆”映射规则。
 */
struct FactorLeverageSpec {
    std::string input_topic;   ///< 原因子 topic
    std::string output_topic;  ///< 杠杆 topic
    std::size_t window = SlidingGaussianLeverage::kDefaultWindow;
};

/**
 * @brief 订阅 DataBus 因子值并以滑窗高斯化方式发布杠杆。
 *
 * - 对每个 (topic, code) 维护独立的 SlidingGaussianLeverage；
 * - 订阅回调仅做事件入队，实际计算在后台线程完成，避免 DataBus 锁重入；
 * - 输出继续通过 DataBus::safe_publish<double>() 发布。
 */
class FactorLeverageTransformer {
public:
    FactorLeverageTransformer(std::vector<FactorLeverageSpec> specs,
                              std::vector<std::string> codes,
                              std::size_t output_capacity = 2048);
    ~FactorLeverageTransformer();

    FactorLeverageTransformer(const FactorLeverageTransformer&) = delete;
    FactorLeverageTransformer& operator=(const FactorLeverageTransformer&) = delete;

    void start();
    void stop();
    bool running() const { return _running; }

    std::optional<SlidingGaussianLeverage::DistributionSnapshot> stats(
        const std::string& input_topic,
        const std::string& code) const;

private:
    struct Event {
        std::size_t spec_index{};
        std::string code;
        int64_t ts_ms{};
        double value{};
    };

    struct TransformState {
        explicit TransformState(std::size_t window) : leverage(window) {}
        SlidingGaussianLeverage leverage;
    };

    static std::string make_state_key(const std::string& topic, const std::string& code) {
        return topic + "|" + code;
    }

    void enqueue_event(std::size_t spec_index,
                       const std::string& code,
                       int64_t ts_ms,
                       double value);
    void worker_loop();
    void process_event(const Event& evt);

    std::vector<FactorLeverageSpec> _specs;
    std::vector<std::string> _codes;
    std::size_t _output_capacity;

    mutable std::mutex _state_mtx;
    std::unordered_map<std::string, TransformState> _states;

    mutable std::mutex _queue_mtx;
    std::condition_variable _queue_cv;
    std::deque<Event> _queue;
    bool _running = false;
    bool _stop_requested = false;
    std::thread _worker;
};

} // namespace factorlib::tools
