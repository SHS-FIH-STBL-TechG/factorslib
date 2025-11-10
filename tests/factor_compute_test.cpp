// tests/factor_compute_test.cpp
// 说明：计算类（factor/compute）测试的总入口。
// - 只包含 GoogleTest 的 main()
// - 其它所有“计算类”测试（如 tick_trans_orders_test.cpp 等）作为独立编译单元加入同一测试目标即可。
// - 如果将来有“功能类”的测试，请放在另一个目标/入口文件（例如 feature_func_test.cpp）。

#include <gtest/gtest.h>
#include "gtest_printer_zh.h"
// 如需在测试启动前后做全局初始化/清理，可在此添加自定义环境：
// struct GlobalEnv : public ::testing::Environment {
//     void SetUp() override {
//         // TODO: 例如加载配置、初始化日志等
//     }
//     void TearDown() override {
//         // TODO
//     }
// };
// 然后在 main 里： ::testing::AddGlobalTestEnvironment(new GlobalEnv());

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gtest_cn::InstallCnDefaultStyle();
    // ::testing::AddGlobalTestEnvironment(new GlobalEnv());
    return RUN_ALL_TESTS();
}
