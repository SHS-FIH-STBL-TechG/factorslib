// numcount_simulation.cpp
#include "NumCountSimulation.h"

NumCountSimulation::NumCountSimulation()
    : _snapshotSHCount(0), _snapshotSZCount(0), _ordAndExeCount(0) {}

NumCountSimulation::~NumCountSimulation() {}

int NumCountSimulation::Init(const std::vector<std::shared_ptr<factorlib::IFactor>>& factors)
{
    // 初始化因子（假设这里直接创建了默认因子对象）
    // 因为我们不使用其他的头文件，因子库的实现由外部提供
    // _factors = factorlib::create_default_factors();  // 这里示范了因子库初始化，但具体代码需要根据外部因子库的实现来确定
    if (!factors.empty()) _factors = factors;
    factorlib::bridge::set_factors(_factors);  // 将因子传入因子库

    // 如果有需要的订阅，可以在此添加

    return 0;  // 返回成功
}

void NumCountSimulation::Finish()
{
    // 清理操作
}

void NumCountSimulation::SimulateSnapSHData(const std::vector<factorlib::QuoteDepth>& data)
{
    // 使用新增的 ingress overload 把已经转换好的 QuoteDepth 发送给 registered factors
    factorlib::bridge::ingest_snapshot_sh(data);
}

void NumCountSimulation::SimulateSnapSZData(const std::vector<factorlib::QuoteDepth>& data)
{
    factorlib::bridge::ingest_snapshot_sz(data);
}

void NumCountSimulation::SimulateOntData(const std::vector<factorlib::CombinedTick>& data)
{
    factorlib::bridge::ingest_ont(data);
}
