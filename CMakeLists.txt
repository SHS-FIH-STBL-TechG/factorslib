# 顶层 CMake 配置：生成 factorlib 库与测试可执行文件
cmake_minimum_required(VERSION 3.15)

project(factorlib LANGUAGES CXX)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 基本编译选项
if (MSVC)
  add_compile_options(/utf-8)  # Ensure source is treated as UTF-8 on MSVC
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_compile_options(/Zc:__cplusplus)  # 让 __cplusplus 正确报告 C++17
else()
  add_compile_options(-Wall -Wextra -O2)
endif()

# ---------- 第三方：googletest ----------
# 需要把官方源码放到 third_party/googletest/googletest/{include,src}
add_subdirectory(third_party/googletest)

# ---------- 工程内公共头路径（也可逐目标设置） ----------
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ---------- 组件：工具库（日志 + 通用工具） ----------
add_library(factorlib_utils
        src/utils/log.cpp
        src/utils/utils.cpp
)
target_include_directories(factorlib_utils PUBLIC include)

# ---------- 组件：基础因子库（示例：tick_trans_orders） ----------
add_library(factor_basic
        src/basic_factors/tick_trans_orders.cpp
        include/ifactor.h
        include/factor_manager.h
)
target_link_libraries(factor_basic PUBLIC factorlib_utils)
target_include_directories(factor_basic PUBLIC include)

# ---------- （可选）spdlog：header-only 检测 ----------
# 把 spdlog 的 include/ 放到 third_party/spdlog/include/ 即可
set(SPDLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
if (EXISTS "${SPDLOG_INCLUDE_DIR}/spdlog/spdlog.h")
  message(STATUS "spdlog headers detected: ${SPDLOG_INCLUDE_DIR}")

  # 让日志在“编译期”不被裁剪（TRACE/DEBUG 也会编译）
  target_compile_definitions(factorlib_utils PUBLIC USE_SPDLOG=1 SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
  target_compile_definitions(factor_basic    PUBLIC USE_SPDLOG=1 SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

  # 头文件路径
  target_include_directories(factorlib_utils PUBLIC ${SPDLOG_INCLUDE_DIR})
  target_include_directories(factor_basic    PUBLIC ${SPDLOG_INCLUDE_DIR})
else()
  message(STATUS "spdlog headers NOT found (fallback to fprintf logger)")
endif()

# ---------- 测试 ----------
enable_testing()

set(FACTORLIB_TEST_SOURCES
        tests/tick_trans_orders_test.cpp
        tests/test_wait.cpp
        tests/factor_compute_test.cpp
        include/ifactor.h
        include/factor_manager.h
        src/utils/data_adapter.cpp
        include/utils/data_adapter.h
)

add_executable(run_tests ${FACTORLIB_TEST_SOURCES})
target_link_libraries(run_tests PRIVATE
        factor_basic
        factorlib_utils
        gtest
        gtest_main
)

# Windows 下不要链接 pthread；非 Windows 下用 Threads::Threads
if (NOT MSVC)
  find_package(Threads REQUIRED)
  target_link_libraries(run_tests PRIVATE Threads::Threads)
endif()

# 若存在 spdlog 头文件，也给 run_tests 加上包含路径与宏（保证测试里也能看到 DEBUG/TRACE）
if (EXISTS "${SPDLOG_INCLUDE_DIR}/spdlog/spdlog.h")
  target_include_directories(run_tests PRIVATE ${SPDLOG_INCLUDE_DIR})
  target_compile_definitions(run_tests PRIVATE USE_SPDLOG=1 SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
endif()

add_test(NAME run_all COMMAND run_tests)
