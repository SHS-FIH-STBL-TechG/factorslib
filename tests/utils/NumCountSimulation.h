// numcount_simulation.h
#pragma once
#include <vector>
#include <memory>
#include <string>
#include "bridge/ingress.h"
#include "core/databus.h"
#include "core/types.h"

namespace factorlib {
    class IFactor; // 因子接口
}

// 定义模拟的类 NumCountSimulation，继承自 Application（如有需要可根据需求调整）
class NumCountSimulation
{
public:
    // 构造函数与析构函数
    NumCountSimulation();
    virtual ~NumCountSimulation();

    // 初始化函数
    // 可选地传入要注册到 bridge 的因子列表（测试中把因子注册到 ingress）
    int Init(const std::vector<std::shared_ptr<factorlib::IFactor>>& factors = {});

    // 结束函数
    void Finish();

    // 模拟接收上交所/深交所快照数据（原始 demo 类型）
    // 模拟接收上交所/深交所快照数据（接受已转换的 QuoteDepth）
    void SimulateSnapSHData(const std::vector<factorlib::QuoteDepth>& data);
    void SimulateSnapSZData(const std::vector<factorlib::QuoteDepth>& data);

    // 模拟接收逐笔交易数据（接受已转换的 CombinedTick）
    void SimulateOntData(const std::vector<factorlib::CombinedTick>& data);

private:
    size_t _snapshotSHCount;  // 上交所快照数据计数
    size_t _snapshotSZCount;  // 深交所快照数据计数
    size_t _ordAndExeCount;   // 逐笔交易数据计数

    std::vector<std::shared_ptr<factorlib::IFactor>> _factors;  // 因子库
};
