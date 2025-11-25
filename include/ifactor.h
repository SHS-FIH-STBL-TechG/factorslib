// include/ifactor.h
#pragma once
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <cstdint>
#include <algorithm>
#include "utils/types.h"
#include "utils/processing_axes.h"
#include "utils/scope_key.h"

namespace factorlib {

    /**
     * @brief 因子接口基类
     * 所有因子都应该继承这个基类，确保统一的接口规范
     */
    class IFactor {
    public:
        virtual ~IFactor() = default;

        // 核心数据处理接口
        virtual void on_quote(const QuoteDepth& q) = 0;
        // virtual void on_transaction(const Transaction& t) = 0;
        // virtual void on_entrust(const Entrust& e) = 0;
        virtual void on_tick(const CombinedTick& x) = 0;
        void on_tick(const Transaction& t) { on_tick(CombinedTick(t)); }
        void on_tick(const Entrust& e)     { on_tick(CombinedTick(e)); }
        virtual void on_bar(const Bar&) {}

        // 强制刷新接口
        virtual bool force_flush(const std::string& code) = 0;

        // 获取因子名称（用于日志和监控）
        virtual std::string get_name() const = 0;

        // 获取监控的股票代码列表
        virtual const std::vector<std::string>& get_codes() const = 0;
    };

    /**
     * @brief 基础因子抽象类
     * 提供时间桶聚合的通用实现
     */
    class BaseFactor : public IFactor {
    protected:
        std::string _name;
        std::vector<std::string> _codes;
        // 记录已初始化过的 code，避免重复初始化
        std::unordered_set<std::string> _known_codes;
        std::vector<int> _custom_freqs;

        struct FrequencyState {
            int frequency{1};
            uint64_t counter{0};
        };

        // 针对每个 code 维护各频率的计数器，避免每次事件都重新构造
        std::unordered_map<std::string, std::vector<FrequencyState>> _freq_states;

        template<typename Fn>
        void for_each_scope(const std::string& code,
                            const std::vector<int>& windows,
                            Fn&& fn) {
            auto& freq_states = _freq_states[code];
            const auto& configured = _custom_freqs.empty() ? get_time_frequencies() : _custom_freqs;
            bool mismatch = freq_states.size() != configured.size();
            if (!mismatch) {
                for (size_t i = 0; i < configured.size(); ++i) {
                    if (freq_states[i].frequency != configured[i]) {
                        mismatch = true;
                        break;
                    }
                }
            }
            if (mismatch) {
                // 频率配置发生变化时，重新同步本地缓存，确保热更新生效
                freq_states.clear();
                freq_states.reserve(configured.size());
                for (int f : configured) {
                    freq_states.push_back(FrequencyState{f, 0});
                }
            }
            if (freq_states.empty()) return;

            const std::vector<int>* window_ptr = &windows;
            std::vector<int> fallback;
            if (windows.empty()) {
                // 默认会传入窗口列表；若因子本身无窗口概念，则生成 {0}
                fallback.push_back(0);
                window_ptr = &fallback;
            }

            for (auto& fs : freq_states) {
                fs.counter++;
                // 频率 N 表示“每 N 条事件触发一次”，非触发周期直接跳过
                if (fs.frequency > 1 && (fs.counter % fs.frequency) != 0) continue;
                for (int window : *window_ptr) {
                    ScopeKey scope{code, fs.frequency, window};
                    fn(scope);
                }
            }
        }
    public:
        BaseFactor(const std::string& name, std::vector<std::string> codes)
            : _name(name), _codes(std::move(codes)) {}

        std::string get_name() const override { return _name; }
        const std::vector<std::string>& get_codes() const override { return _codes; }

    protected:
        /**
         * @brief 确保某个 code 的内部状态已初始化（只在首次见到该 code 时触发）
         * 典型用途：
         *  - 为该 code 创建窗口/缓存/统计器
         *  - 注册 DataBus topic 的订阅/发布钩子
         *  - 建立跨模块的索引（如 code->state 映射）
         */
        void ensure_code(const std::string& code) {
            if (_known_codes.find(code) != _known_codes.end()) return;
            _known_codes.insert(code);
            on_code_added(code);
        }

        /**
         * @brief 派生类可覆盖此钩子，完成 code 级别的自定义初始化
         * 缺省实现为空。
         */
        virtual void on_code_added(const std::string& /*code*/) {}

        /// @brief 为当前因子设置自定义频率列表（如配置中指定了 time_frequencies）
        void set_time_frequencies_override(std::vector<int> freqs) {
            freqs.erase(std::remove_if(freqs.begin(), freqs.end(),
                                       [](int f) { return f <= 0; }),
                        freqs.end());
            if (freqs.empty()) {
                _custom_freqs.clear();
                return;
            }
            std::sort(freqs.begin(), freqs.end());
            freqs.erase(std::unique(freqs.begin(), freqs.end()), freqs.end());
            _custom_freqs = std::move(freqs);
        }
    };

} // namespace factorlib
