// include/ifactor.h
#pragma once
#include <string>
#include <vector>
#include <unordered_set>
#include "utils/types.h"

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
        virtual void on_transaction(const Transaction& t) = 0;
        virtual void on_entrust(const Entrust& e) = 0;

        // 强制刷新接口
        virtual void on_bar(const Bar& b) {}

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
        std::vector<std::string> _codes;
        std::string _name;
        // 记录已初始化过的 code，避免重复初始化
        std::unordered_set<std::string> _known_codes;
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
    };

} // namespace factorlib