#include <gtest/gtest.h>
#include "gtest_printer_zh.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gtest_cn::InstallCnDefaultStyle();
    return RUN_ALL_TESTS();
}
