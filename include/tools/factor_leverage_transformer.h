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
 * @brief 描述一条"因子 → 杠杆"映射规则。
 * 
 * 用于配置 FactorLeverageTransformer，指定输入因子 topic 和输出杠杆 topic。
 */
struct FactorLeverageSpec {
    std::string input_topic;   ///< 原因子 topic（从 DataBus 订阅）
    std::string output_topic;  ///< 杠杆 topic（转换后发布到 DataBus）
    std::size_t window = SlidingGaussianLeverage::kDefaultWindow;  ///< 滑动窗口大小
};

/**
 * @brief 订阅 DataBus 因子值并以滑窗高斯化方式发布杠杆。
 *
 * 核心功能：实时将因子原始值转换为标准化的 z-score 杠杆信号
 * 
 * 工作流程：
 * 1. 启动时从 DataBus 订阅所有配置的 (input_topic, code) 组合
 * 2. 接收到因子值时，加入事件队列（避免在回调中直接计算，防止死锁）
 * 3. 后台工作线程处理事件队列：
 *    - 使用对应的 SlidingGaussianLeverage 计算 z-score
 *    - 将结果发布到 output_topic
 * 4. 为每个 (input_topic, code) 维护独立的滑动窗口状态
 * 
 * 设计特点：
 * - 异步处理：订阅回调仅入队，避免 DataBus 锁重入
 * - 状态隔离：每个因子-股票组合独立维护统计状态
 * - 线程安全：使用互斥锁保护共享状态
 * 
 * 典型用法：
 * @code
 * std::vector<FactorLeverageSpec> specs = {
 *     {"factor/ar1_return", "leverage/ar1_return", 250}
 * };
 * std::vector<std::string> codes = {"000001.SZ", "600000.SH"};
 * FactorLeverageTransformer transformer(specs, codes);
 * transformer.start();
 * // ... 系统运行 ...
 * transformer.stop();
 * @endcode
 */
class FactorLeverageTransformer {
public:
    /**
     * @brief 构造函数
     * @param specs 因子到杠杆的映射规则列表
     * @param codes 股票代码列表
     * @param output_capacity 输出 topic 的容量（每个 topic 的缓冲区大小）
     */
    FactorLeverageTransformer(std::vector<FactorLeverageSpec> specs,
                              std::vector<std::string> codes,
                              std::size_t output_capacity = 2048);
    ~FactorLeverageTransformer();

    FactorLeverageTransformer(const FactorLeverageTransformer&) = delete;
    FactorLeverageTransformer& operator=(const FactorLeverageTransformer&) = delete;

    /**
     * @brief 启动转换器
     * 
     * 功能：
     * 1. 在 DataBus 上注册所有输出 topic
     * 2. 订阅所有配置的输入 topic
     * 3. 启动后台工作线程
     */
    void start();
    
    /**
     * @brief 停止转换器
     * 
     * 功能：
     * 1. 取消所有订阅
     * 2. 等待事件队列处理完毕
     * 3. 停止工作线程
     */
    void stop();
    
    /**
     * @brief 检查转换器是否正在运行
     * @return true 表示正在运行
     */
    bool running() const { return _running; }

    /**
     * @brief 获取指定因子-股票的统计信息
     * @param input_topic 输入因子 topic
     * @param code 股票代码
     * @return 统计快照，若该组合不存在则返回 nullopt
     */
    std::optional<SlidingGaussianLeverage::DistributionSnapshot> stats(
        const std::string& input_topic,
        const std::string& code) const;

private:
    /**
     * @brief 事件结构，表示一次因子值更新
     */
    struct Event {
        std::size_t spec_index{};  ///< 对应的 spec 索引
        std::string code;          ///< 股票代码
        int64_t ts_ms{};           ///< 时间戳（毫秒）
        double value{};            ///< 因子值
    };

    /**
     * @brief 单个 (topic, code) 的转换状态
     */
    struct TransformState {
        explicit TransformState(std::size_t window) : leverage(window) {}
        SlidingGaussianLeverage leverage;  ///< 滑窗高斯化转换器
    };

    /**
     * @brief 生成状态键：topic|code
     * @param topic 因子 topic
     * @param code 股票代码
     * @return 唯一标识该组合的字符串键
     */
    static std::string make_state_key(const std::string& topic, const std::string& code) {
        return topic + "|" + code;
    }

    /**
     * @brief 将因子值事件加入队列
     * @param spec_index 规则索引
     * @param code 股票代码
     * @param ts_ms 时间戳
     * @param value 因子值
     * 
     * 该函数在 DataBus 订阅回调中调用，只做入队操作，不阻塞。
     */
    void enqueue_event(std::size_t spec_index,
                       const std::string& code,
                       int64_t ts_ms,
                       double value);
    
    /**
     * @brief 工作线程主循环
     * 
     * 功能：
     * 1. 等待事件队列有数据
     * 2. 取出事件并处理
     * 3. 收到停止信号且队列为空时退出
     */
    void worker_loop();
    
    /**
     * @brief 处理单个事件
     * @param evt 事件对象
     * 
     * 功能：
     * 1. 查找或创建对应的 TransformState
     * 2. 调用 SlidingGaussianLeverage::transform() 计算 z-score
     * 3. 将结果发布到输出 topic
     */
    void process_event(const Event& evt);

    std::vector<FactorLeverageSpec> _specs;  ///< 转换规则列表
    std::vector<std::string> _codes;         ///< 股票代码列表
    std::size_t _output_capacity;            ///< 输出容量

    mutable std::mutex _state_mtx;           ///< 保护 _states 的互斥锁
    std::unordered_map<std::string, TransformState> _states;  ///< (topic|code) -> 转换状态

    mutable std::mutex _queue_mtx;           ///< 保护事件队列的互斥锁
    std::condition_variable _queue_cv;       ///< 事件队列条件变量
    std::deque<Event> _queue;                ///< 事件队列
    bool _running = false;                   ///< 运行状态标志
    bool _stop_requested = false;            ///< 停止请求标志
    std::thread _worker;                     ///< 后台工作线程
};

} // namespace factorlib::tools
