// tests/test_wait.cpp
#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "utils/databus.h"

/**
 * @file test_wait.cpp
 * @brief 用例目标：验证“阻塞等待”能力：
 *   - 先在子线程延迟 100ms 发布一条 (topic, code, ts) 的数据；
 *   - 主线程调用 wait_for_time_exact(..., timeout=1000ms)；
 *   - 预期：不超时且拿到正确数据。
 */

using namespace factorlib;

TEST(DataBusWait, Exact){
    auto& bus = DataBus::instance();
    bus.register_topic<double>("demo/wait_price", 8);
    std::string code="000001.SZ";
    int64_t ts=1234;
    double out=0;
    // 子线程 100ms 后发布
    std::thread th([&](){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bus.publish<double>("demo/wait_price", code, ts, 11.11);
    });
    bool ok = bus.wait_for_time_exact<double>("demo/wait_price", code, ts, out, 1000);
    th.join();
    ASSERT_TRUE(ok);
    EXPECT_DOUBLE_EQ(out, 11.11);
}
