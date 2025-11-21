// src/factor_factory.cpp
#include "factor_factory.h"

#include "ifactor.h"
#include "basic_factors/tick_trans_orders.h"
#include "stat_factors/granger_causality_factor.h"

namespace factorlib {

std::vector<std::shared_ptr<IFactor>> create_default_factors() {
    std::vector<std::shared_ptr<IFactor>> factors;

    std::vector<std::string> codes; // 为空时，因子内部通常对所有代码生效

    TickTransOrdersConfig t_cfg;
    auto tto = std::make_shared<TickTransOrders>(t_cfg, codes);

    GrangerConfig g_cfg;
    auto granger = std::make_shared<GrangerCausalityFactor>(codes, g_cfg);

    // 静态注册所有相关 topic（容量可根据需要调整）
    TickTransOrders::register_topics(120);
    GrangerCausalityFactor::register_topics(1024);

    factors.emplace_back(std::move(tto));
    factors.emplace_back(std::move(granger));
    return factors;
}

} // namespace factorlib
