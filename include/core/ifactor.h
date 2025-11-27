// include/core/ifactor.h
#pragma once
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <cstdint>
#include <algorithm>
#include "core/types.h"
#include "core/scope_key.h"
#include "utils/log.h"

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

        static constexpr size_t K_MAX_WINDOW_OPTIONS = 8;
        static constexpr int    K_MAX_WINDOW_SIZE = 500;

        template<typename Fn>
        void for_each_scope(const std::string& code,
                            const std::vector<int>& windows,
                            int64_t ts_ms,
                            Fn&& fn) {
            (void)ts_ms;

            const std::vector<int>* window_ptr = &windows;
            std::vector<int> fallback;
            if (windows.empty()) {
                // 默认会传入窗口列表；若因子本身无窗口概念，则生成 {0}
                fallback.push_back(0);
                window_ptr = &fallback;
            }

            for (int window : *window_ptr) {
                ScopeKey scope{code, window};
                fn(scope);
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

        template<typename T>
        void clamp_list_size(std::vector<T>& values, size_t max_count, const char* label) const {
            if (max_count == 0 || values.size() <= max_count) return;
            LOG_WARN("{}: 配置数量 {} 超过上限 {}，仅保留前 {} 项",
                     label ? label : "配置", values.size(), max_count, max_count);
            values.resize(max_count);
        }

        void clamp_window_list(std::vector<int>& windows, const char* label) const {
            clamp_list_size(windows, K_MAX_WINDOW_OPTIONS, label);
            for (auto& w : windows) {
                if (w > K_MAX_WINDOW_SIZE) {
                    LOG_WARN("{}: 窗口大小 {} 超过上限 {}，自动截断为 {}",
                             label ? label : "window_sizes", w, K_MAX_WINDOW_SIZE, K_MAX_WINDOW_SIZE);
                    w = K_MAX_WINDOW_SIZE;
                }
                if (w <= 0) w = 1;
            }
        }

    };

} // namespace factorlib
