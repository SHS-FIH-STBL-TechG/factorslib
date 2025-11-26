// tests/utils/databus_test.cpp
// 目标：覆盖 DataBus 的基础行为，包括发布/读取/订阅/等待，以及裸 code 解析。
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>

#include "utils/databus.h"
#include "utils/scope_key.h"

using namespace factorlib;

namespace {

struct BusGuard {
    BusGuard()  { DataBus::instance().reset(); }
    ~BusGuard() { DataBus::instance().reset(); }
};

constexpr const char* kTopic = "unittest/databus";

} // namespace

// 验证：注册后能按时间顺序读到最新值
TEST(DataBusTest, PublishAndGetLatest) {
    BusGuard guard;
    auto& bus = DataBus::instance();
    bus.register_topic<double>(kTopic, 8);

    double out = 0.0;
    int64_t ts = 0;
    EXPECT_FALSE(bus.get_latest<double>(kTopic, "CODE", out, &ts));

    bus.publish<double>(kTopic, "CODE", 100, 1.25);
    ASSERT_TRUE(bus.get_latest<double>(kTopic, "CODE", out, &ts));
    EXPECT_EQ(ts, 100);
    EXPECT_DOUBLE_EQ(out, 1.25);

    bus.publish<double>(kTopic, "CODE", 120, 2.5);
    ASSERT_TRUE(bus.get_latest<double>(kTopic, "CODE", out, &ts));
    EXPECT_EQ(ts, 120);
    EXPECT_DOUBLE_EQ(out, 2.5);
}

// 验证：只传 code 时仍能命中 `code|wX` 的组合键
TEST(DataBusTest, BareCodeResolvesWindowScope) {
    BusGuard guard;
    auto& bus = DataBus::instance();
    bus.register_topic<double>(kTopic, 4);

    const std::string code = "SH600000";
    const std::string scoped = compose_scope_code(code, 60);
    bus.publish<double>(kTopic, scoped, 500, 3.14);

    double out = 0.0;
    int64_t ts = 0;
    ASSERT_TRUE(bus.get_latest<double>(kTopic, code, out, &ts));
    EXPECT_EQ(ts, 500);
    EXPECT_DOUBLE_EQ(out, 3.14);

    double exact = 0.0;
    ASSERT_TRUE(bus.get_by_time_exact<double>(kTopic, code, 500, exact));
    EXPECT_DOUBLE_EQ(exact, 3.14);
}

// 验证：容量淘汰逻辑以及 get_last_n 的顺序
TEST(DataBusTest, HistoryCapacityAndOrdering) {
    BusGuard guard;
    auto& bus = DataBus::instance();
    bus.register_topic<double>(kTopic, 2);

    bus.publish<double>(kTopic, "CODE", 10, 1.0);
    bus.publish<double>(kTopic, "CODE", 20, 2.0);
    bus.publish<double>(kTopic, "CODE", 30, 3.0);

    const auto rows = bus.get_last_n<double>(kTopic, "CODE", 5);
    ASSERT_EQ(rows.size(), 2u);
    EXPECT_EQ(rows[0].first, 20);
    EXPECT_DOUBLE_EQ(rows[0].second, 2.0);
    EXPECT_EQ(rows[1].first, 30);
    EXPECT_DOUBLE_EQ(rows[1].second, 3.0);
}

// 验证：订阅回调能按发布顺序获取最新值
TEST(DataBusTest, SubscriptionReceivesLatestValue) {
    BusGuard guard;
    auto& bus = DataBus::instance();
    bus.register_topic<double>(kTopic, 4);

    std::atomic<int> hits{0};
    std::string last_code;
    int64_t last_ts = 0;
    double last_value = 0.0;

    bus.subscribe<double>(kTopic, "CODE",
        [&](const std::string& code, int64_t ts, const double& value) {
            hits.fetch_add(1, std::memory_order_relaxed);
            last_code = code;
            last_ts = ts;
            last_value = value;
        });

    bus.publish<double>(kTopic, "CODE", 42, 6.28);
    EXPECT_EQ(hits.load(std::memory_order_relaxed), 1);
    EXPECT_EQ(last_code, "CODE");
    EXPECT_EQ(last_ts, 42);
    EXPECT_DOUBLE_EQ(last_value, 6.28);
}

// 验证：wait_for_time_at_least 会返回第一条时间戳 >= 目标值的数据
TEST(DataBusTest, WaitForTimeAtLeastReturnsFirstNewerTick) {
    BusGuard guard;
    auto& bus = DataBus::instance();
    bus.register_topic<double>(kTopic, 4);

    double out = 0.0;
    std::thread producer([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bus.publish<double>(kTopic, "CODE", 100, 1.0);
    });

    bool ok = bus.wait_for_time_at_least<double>(kTopic, "CODE", 80, out, 500);
    producer.join();
    ASSERT_TRUE(ok);
    EXPECT_DOUBLE_EQ(out, 1.0);
}
