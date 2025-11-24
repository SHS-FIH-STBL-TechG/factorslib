// include/factorlib/bridge/ingress.h
#pragma once
#include <vector>
#include <memory>
#include "ifactor.h"

/**
 * @file ingress.h
 * @brief demo → factorlib 的入口层（最小入侵）：
 *        demo 在各自回调中只需调用 ingest_* 一行，转化与分发均在库内完成。
 */

struct std_SnapshotStockSH;
struct std_SnapshotStockSZ;
struct std_OrdAndExeInfo;
struct std_BasicandEnhanceKLine;

namespace factorlib::bridge {

// 程序启动时注册因子集合（demo 初始化时调用一次）
void set_factors(const std::vector<std::shared_ptr<factorlib::IFactor>>& factors);

// demo 的回调一行调用
void ingest_snapshot_sh(const std::vector<std_SnapshotStockSH>& v);
void ingest_snapshot_sz(const std::vector<std_SnapshotStockSZ>& v);
void ingest_ont(const std::vector<std_OrdAndExeInfo>& v);
void ingest_kline(const std::vector<std_BasicandEnhanceKLine>& v);

// Overloads for tests: accept already-converted types and dispatch to registered factors.
void ingest_snapshot_sh(const std::vector<factorlib::QuoteDepth>& v);
void ingest_snapshot_sz(const std::vector<factorlib::QuoteDepth>& v);
void ingest_ont(const std::vector<factorlib::CombinedTick>& v);

} // namespace factorlib::bridge
