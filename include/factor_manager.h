// include/factor_manager.h
#pragma once
#include <memory>
#include <unordered_map>
#include "core/ifactor.h"

namespace factorlib {

    class FactorManager {
    public:
        static FactorManager& instance();

        void register_factor(std::shared_ptr<IFactor> factor);
        void unregister_factor(const std::string& name);

        // 批量处理数据
        void on_quote(const QuoteDepth& q);
        void on_transaction(const Transaction& t);
        void on_entrust(const Entrust& e);

        void force_flush_all();

    private:
        std::unordered_map<std::string, std::shared_ptr<IFactor>> _factors;
    };

} // namespace factorlib