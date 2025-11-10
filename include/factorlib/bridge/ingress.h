#pragma once
#include <vector>
#include <memory>
#include "../../demo_header/DataType.h"
#include "ifactor.h"

/**
 * @file ingress.h
 * @brief demo → factorlib 的入口层（最小入侵）：
 *        demo 在各自回调中只需调用 ingest_* 一行，转化与分发均在库内完成。
 */
namespace factorlib::bridge {

// 程序启动时注册因子集合（demo 初始化时调用一次）
void set_factors(const std::vector<std::shared_ptr<factorlib::IFactor>>& factors);

// demo 的回调一行调用
void ingest_snapshot(const std::vector<SnapshotStockSH>& v);
void ingest_ont(const std::vector<OrdAndExeInfo>& v);
void ingest_kline(const std::vector<BasicandEnhanceKLine>& v);

} // namespace factorlib::bridge
