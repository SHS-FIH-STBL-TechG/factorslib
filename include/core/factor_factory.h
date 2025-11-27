// include/core/factor_factory.h
#pragma once
#include <memory>
#include <vector>

namespace factorlib {

class IFactor;

/// 平台入口：创建默认的因子集合（例如 TickTransOrders / GrangerCausalityFactor）
/// 实现细节封装在因子库内部，调用侧只需要拿到 IFactor 指针并通过 bridge::set_factors 注册。
std::vector<std::shared_ptr<IFactor>> create_default_factors();

} // namespace factorlib
