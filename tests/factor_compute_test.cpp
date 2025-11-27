// tests/factor_compute_test.cpp
// 说明：计算类（factor/compute）测试的总入口。
// - 只包含 GoogleTest 的 main()
// - 其它所有"计算类"测试（如 tick_trans_orders_test.cpp 等）作为独立编译单元加入同一测试目标即可。
// - 如果将来有"功能类"的测试，请放在另一个目标/入口文件（例如 feature_func_test.cpp）。

#include <gtest/gtest.h>
#include "gtest_printer_zh.h"
#include "instrumentation/trace_helper.h"
#include <iostream>

// 全局环境：在所有测试开始前初始化 Perfetto，结束后关闭
struct GlobalEnv : public ::testing::Environment {
    void SetUp() override {
        // 初始化 Perfetto 追踪
        bool trace_enabled = factorlib::trace::TraceHelper::initialize("factor_tests.pftrace");
        if (trace_enabled) {
            std::cout << "[追踪] Perfetto 已启用，trace 将保存到: "
                      << factorlib::trace::TraceHelper::current_trace_path() << std::endl;
        }
    }

    void TearDown() override {
        // 关闭追踪并保存文件
        if (factorlib::trace::TraceHelper::is_enabled()) {
            factorlib::trace::TraceHelper::shutdown();
            std::cout << "[追踪] Trace 已保存到: "
                      << factorlib::trace::TraceHelper::current_trace_path()
                      << "，在 https://ui.perfetto.dev 查看" << std::endl;
        }
    }
};

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gtest_cn::InstallCnDefaultStyle();

    // 添加全局测试环境（包含 Perfetto 初始化/关闭）
    ::testing::AddGlobalTestEnvironment(new GlobalEnv());

    return RUN_ALL_TESTS();
}
