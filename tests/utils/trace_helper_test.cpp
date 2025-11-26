// tests/utils/trace_helper_test.cpp
#include <gtest/gtest.h>
#include "utils/trace_helper.h"
#include "bridge/ingress.h"
#include "utils/types.h"
#include "stat_factors/volume_mgf_factor.h"
#include <vector>
#include <thread>
#include <chrono>
#include <fstream>
#include <algorithm>

using namespace factorlib;
using namespace factorlib::trace;

class TraceHelperTest : public ::testing::Test {
protected:
    void SetUp() override {
        was_enabled_before_ = TraceHelper::is_enabled();
        if (was_enabled_before_) {
            TraceHelper::shutdown();
        }
    }

    void TearDown() override {
        // 确保追踪系统关闭
        TraceHelper::shutdown();
        if (was_enabled_before_) {
            // 还原全局环境的初始化状态，避免影响其它测试
            TraceHelper::initialize("factor_tests.pftrace");
        }
    }

private:
    bool was_enabled_before_ = false;
};

TEST_F(TraceHelperTest, InitializeAndShutdown) {
    // 测试初始化
    bool init_result = TraceHelper::initialize("test_trace.pftrace");

#ifdef FACTORLIB_ENABLE_PERFETTO
    EXPECT_TRUE(init_result);
    EXPECT_TRUE(TraceHelper::is_enabled());
#else
    EXPECT_FALSE(init_result);
    EXPECT_FALSE(TraceHelper::is_enabled());
#endif

    // 测试关闭
    TraceHelper::shutdown();
    EXPECT_FALSE(TraceHelper::is_enabled());
}

TEST_F(TraceHelperTest, TraceEventBasic) {
    TraceHelper::initialize("test_event.pftrace");

    // 添加一些追踪事件
    for (int i = 0; i < 10; ++i) {
        uint64_t unique_id = 1000 + i;
        TraceHelper::trace_event("test", "test_event", "000001.SZ", 60, unique_id);
    }

    TraceHelper::shutdown();

    // 如果启用了 Perfetto，文件应该存在
#ifdef FACTORLIB_ENABLE_PERFETTO
    std::ifstream file("test_event.pftrace", std::ios::binary);
    EXPECT_TRUE(file.good());
    file.close();
#endif
}

TEST_F(TraceHelperTest, TraceScopeRAII) {
    TraceHelper::initialize("test_scope.pftrace");

    {
        TraceScope scope("test", "scope_test", "000001.SZ", 60, 12345);
        // 在作用域内执行一些操作
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } // scope 自动结束

    TraceHelper::shutdown();
}

TEST_F(TraceHelperTest, TraceCounter) {
    TraceHelper::initialize("test_counter.pftrace");

    // 模拟计数器变化
    for (int i = 0; i < 20; ++i) {
        TraceHelper::trace_counter("test", "counter_value", "000001.SZ", 60,
                                   static_cast<double>(i * 10));
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    TraceHelper::shutdown();
}

TEST_F(TraceHelperTest, IngressTracing) {
    TraceHelper::initialize("test_ingress.pftrace");

    // 创建测试数据
    std::vector<Bar> bars;
    for (int i = 0; i < 5; ++i) {
        Bar b;
        b.instrument_id = "000001.SZ";
        b.data_time_ms = 1609459200000LL + i * 60000; // 每分钟一个 bar
        b.open = 10.0 + i * 0.1;
        b.high = 10.5 + i * 0.1;
        b.low = 9.5 + i * 0.1;
        b.close = 10.2 + i * 0.1;
        b.volume = 1000 + i * 100;
        bars.push_back(b);
    }

    // 通过 ingress 注入数据（会自动生成 trace）
    factorlib::bridge::ingest_kline(bars);

    TraceHelper::shutdown();
}

TEST_F(TraceHelperTest, FactorComputeTracing) {
    TraceHelper::initialize("test_factor_compute.pftrace");

    // 创建因子
    std::vector<std::string> codes = {"000001.SZ", "000002.SZ"};
    VolumeMGFConfig cfg;
    cfg.window_size = 10;
    cfg.t = 0.01;

    auto factor = std::make_shared<VolumeMGFFactor>(codes, cfg);
    VolumeMGFFactor::register_topics(1000);

    // 设置因子
    std::vector<std::shared_ptr<IFactor>> factors = {factor};
    factorlib::bridge::set_factors(factors);

    // 创建测试数据
    std::vector<Bar> bars;
    for (int i = 0; i < 20; ++i) {
        Bar b;
        b.instrument_id = "000001.SZ";
        b.data_time_ms = 1609459200000LL + i * 60000;
        b.open = 10.0;
        b.high = 10.5;
        b.low = 9.5;
        b.close = 10.2;
        b.volume = 1000 + i * 50;
        bars.push_back(b);
    }

    // 注入数据（会触发因子计算和 trace）
    factorlib::bridge::ingest_kline(bars);

    TraceHelper::shutdown();

#ifdef FACTORLIB_ENABLE_PERFETTO
    // 检查 trace 文件
    std::ifstream file("test_factor_compute.pftrace", std::ios::binary);
    EXPECT_TRUE(file.good());

    // 检查文件大小（应该有实际内容）
    file.seekg(0, std::ios::end);
    auto file_size = file.tellg();
    EXPECT_GT(file_size, 100); // 至少有 100 字节
    file.close();
#endif
}

TEST_F(TraceHelperTest, MultipleWindowsTracing) {
    TraceHelper::initialize("test_multiple_windows.pftrace");

    // 创建具有多个窗口的因子
    std::vector<std::string> codes = {"000001.SZ"};
    VolumeMGFConfig cfg;
    cfg.window_size = 10;
    cfg.t = 0.01;

    auto factor = std::make_shared<VolumeMGFFactor>(codes, cfg);
    VolumeMGFFactor::register_topics(1000);

    std::vector<std::shared_ptr<IFactor>> factors = {factor};
    factorlib::bridge::set_factors(factors);

    // 创建大量数据以触发多个窗口
    std::vector<Bar> bars;
    for (int i = 0; i < 50; ++i) {
        Bar b;
        b.instrument_id = "000001.SZ";
        b.data_time_ms = 1609459200000LL + i * 1000; // 每秒一个
        b.volume = 1000 + (i % 20) * 100;
        bars.push_back(b);
    }

    factorlib::bridge::ingest_kline(bars);

    TraceHelper::shutdown();
}

// 性能测试：确保追踪不会显著降低性能
TEST_F(TraceHelperTest, PerformanceOverhead) {
    const int num_events = 1000;

    // 不启用追踪的情况
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_events; ++i) {
        // 模拟一些工作
        volatile int x = i * i;
        (void)x;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_no_trace = std::chrono::duration<double, std::micro>(end - start).count();

    // 启用追踪的情况
    TraceHelper::initialize("test_performance.pftrace");
    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_events; ++i) {
        FACTORLIB_TRACE_EVENT("perf_test", "test_event", "000001.SZ", 0, i);
        volatile int x = i * i;
        (void)x;
    }
    end = std::chrono::high_resolution_clock::now();
    auto duration_with_trace = std::chrono::duration<double, std::micro>(end - start).count();
    TraceHelper::shutdown();

#ifdef FACTORLIB_ENABLE_PERFETTO
    // 关注单次事件新增的绝对耗时（微秒），避免因基准耗时过小导致比例失真
    double per_event_overhead =
        std::max(0.0, (duration_with_trace - duration_no_trace) / num_events);
    std::cout << "Trace overhead per event: " << per_event_overhead << " us" << std::endl;
    EXPECT_LT(per_event_overhead, 200.0);
#else
    // 未启用时，开销应该几乎为零
    EXPECT_NEAR(duration_with_trace, duration_no_trace, duration_no_trace * 0.1);
#endif
}
