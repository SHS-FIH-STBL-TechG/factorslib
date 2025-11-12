# 顶层 CMake 配置：生成 factorlib 库与测试可执行文件
cmake_minimum_required(VERSION 3.15)

project(factorlib LANGUAGES CXX)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ========================
# 选项
# ========================
option(FACTORLIB_BUILD_TESTS "Build tests" ON)
option(FACTORLIB_USE_THIRD_PARTY "Prefer bundled third_party libs" ON)
option(FACTORLIB_ENABLE_TRACE_DEBUG "Compile TRACE/DEBUG logs" OFF)
option(FACTORLIB_WITH_DEMO_E2E "Build E2E test depending on demo_header" OFF)
option(FACTORLIB_BUILD_UNIT_TESTS "Build factorlib unit tests (non-E2E)" ON)
# ========================
# 编译选项
# ========================
if (MSVC)
  add_compile_options(/utf-8)  # Ensure source is treated as UTF-8 on MSVC
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_compile_options(/Zc:__cplusplus)  # 让 __cplusplus 正确报告 C++17
else()
  add_compile_options(-Wall -Wextra -O2)
endif()

# ========================
# 第三方依赖（优先使用 third_party）
# ========================
# googletest：需要把官方源码放到 third_party/googletest/googletest/{include,src}
if (FACTORLIB_BUILD_TESTS)
  add_subdirectory(third_party/googletest)
endif()

# ---------- 第三方：Eigen ----------
if (FACTORLIB_USE_THIRD_PARTY)
    set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
    if (EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
        message(STATUS "Using bundled Eigen: ${EIGEN3_INCLUDE_DIR}")
        include_directories(${EIGEN3_INCLUDE_DIR})
    else()
        message(WARNING "Eigen headers NOT found at ${EIGEN3_INCLUDE_DIR}")
        message(WARNING "Please download Eigen to third_party/eigen/")
        message(WARNING "Download from: https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz")
    endif()
endif()
# ---------- 工程内公共头路径（也可逐目标设置） ----------
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
#        ${CMAKE_CURRENT_SOURCE_DIR}/../demo_header  # 添加 demo_header 路径
)

# ====== Boost（使用 bcp 导出的最小头，避免 FindBoost / CMP0167） ======
set(BOOST_VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost)
if (EXISTS "${BOOST_VENDOR_DIR}/boost/config.hpp")
    message(STATUS "Using bundled Boost headers (bcp): ${BOOST_VENDOR_DIR}")
else()
    message(FATAL_ERROR "Bundled Boost headers not found at ${BOOST_VENDOR_DIR}.
    请先用 bcp 导出到 factors_lib/third_party/boost，再重新配置。")
endif()

# 统一把 Boost vendor include 加到各目标
set(FACTORLIB_THIRDPARTY_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${BOOST_VENDOR_DIR}
        ${EIGEN3_INCLUDE_DIR}
)


# ---------- 组件：工具库（日志 + 通用工具） ----------
add_library(factorlib_utils
        src/utils/log.cpp
        src/bridge/ingress.cpp
        src/utils/data_adapter.cpp
        src/utils/trading_time.cpp
        src/utils/nms_bucket_aggregator.cpp
        include/utils/math/incremental_rank.h
        include/utils/config/feed_mode.h
        include/utils/math/sliding_normal_eq.h
        # 注意：移除了 math_utils.cpp
)
target_include_directories(factorlib_utils PUBLIC ${FACTORLIB_THIRDPARTY_INCLUDES})

# ---------- 组件：基础因子库（现有因子 + 新因子） ----------
add_library(factor_basic
        src/basic_factors/tick_trans_orders.cpp
        src/gaussian_copula_factor.cpp  # 添加高斯 Copula 因子
        src/granger_causality_factor.cpp
        src/granger_causality_factor.h
        src/granger_causality_factor.cpp
)
target_link_libraries(factor_basic PUBLIC factorlib_utils)
target_include_directories(factor_basic PUBLIC include)

# ---------- （可选）spdlog：header-only 检测 ----------
# 把 spdlog 的 include/ 放到 third_party/spdlog/include/ 即可
set(SPDLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
if (EXISTS "${SPDLOG_INCLUDE_DIR}/spdlog/spdlog.h")
    message(STATUS "spdlog headers detected: ${SPDLOG_INCLUDE_DIR}")
    target_include_directories(factorlib_utils PUBLIC ${SPDLOG_INCLUDE_DIR})
    target_include_directories(factor_basic  PUBLIC ${SPDLOG_INCLUDE_DIR})
    target_compile_definitions(factorlib_utils PUBLIC USE_SPDLOG=1)
    target_compile_definitions(factor_basic  PUBLIC USE_SPDLOG=1)
    # 编译期裁剪日志级别（默认移除 TRACE/DEBUG）
    if (FACTORLIB_ENABLE_TRACE_DEBUG)
        target_compile_definitions(factorlib_utils PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
        target_compile_definitions(factor_basic  PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
    else()
        target_compile_definitions(factorlib_utils PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO FACTORLIB_NO_DEBUG_TRACE=1)
        target_compile_definitions(factor_basic  PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO FACTORLIB_NO_DEBUG_TRACE=1)
    endif()
else()
    message(STATUS "spdlog headers NOT found (fallback to fprintf logger)")
    # 对 fallback 宏同样支持编译期移除 TRACE/DEBUG
    if (NOT FACTORLIB_ENABLE_TRACE_DEBUG)
        target_compile_definitions(factorlib_utils PUBLIC FACTORLIB_NO_DEBUG_TRACE=1)
        target_compile_definitions(factor_basic  PUBLIC FACTORLIB_NO_DEBUG_TRACE=1)
    endif()
endif()

# ---------- 测试 ----------
if (FACTORLIB_BUILD_TESTS AND FACTORLIB_BUILD_UNIT_TESTS)
    enable_testing()
    set(FACTORLIB_TEST_SOURCES
        tests/tick_trans_orders_test.cpp
        tests/test_wait.cpp
        tests/factor_compute_test.cpp
        tests/gaussian_copula_factor_test.cpp
        tests/granger_causality_factor_test.cpp
    )
    add_executable(run_tests ${FACTORLIB_TEST_SOURCES} tests/gtest_printer_zh.h)
    target_link_libraries(run_tests PRIVATE factor_basic factorlib_utils gtest gtest_main )
    # Windows 下不要链接 pthread；非 Windows 下用 Threads::Threads
    if (NOT MSVC)
        find_package(Threads REQUIRED)
        target_link_libraries(run_tests PRIVATE Threads::Threads)
    endif()
    # 若存在 spdlog 头文件，也给 run_tests 加上包含路径与宏（保证测试里也能看到 DEBUG/TRACE）
    if (EXISTS "${SPDLOG_INCLUDE_DIR}/spdlog/spdlog.h")
        target_include_directories(run_tests PRIVATE ${SPDLOG_INCLUDE_DIR})
    if (FACTORLIB_ENABLE_TRACE_DEBUG)
        target_compile_definitions(run_tests PRIVATE USE_SPDLOG=1 SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
    else()
        target_compile_definitions(run_tests PRIVATE USE_SPDLOG=1 SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO FACTORLIB_NO_DEBUG_TRACE=1)
    endif()
    else()
        if (NOT FACTORLIB_ENABLE_TRACE_DEBUG)
          target_compile_definitions(run_tests PRIVATE FACTORLIB_NO_DEBUG_TRACE=1)
        endif()
    endif()
    # 为测试添加 demo_header 路径
    target_include_directories(run_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../demo_header )
    add_test(NAME run_all COMMAND run_tests)
endif()

if (FACTORLIB_WITH_DEMO_E2E)
    enable_testing()

    add_executable(run_e2e tests/integration/demo_min_e2e_test.cpp)
    target_link_libraries(run_e2e PRIVATE factor_basic factorlib_utils gtest gtest_main)
    if (NOT MSVC)
        find_package(Threads REQUIRED)
        target_link_libraries(run_e2e PRIVATE Threads::Threads)
    endif()

    # 优先用顶层提供的 INTERFACE 目标；没有就退回相对路径
    if (TARGET demo_headers)
        target_link_libraries(run_e2e PRIVATE demo_headers)
    else()
        target_include_directories(run_e2e PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../demo_header)
    endif()

    # spdlog 宏同步
    if (EXISTS "${SPDLOG_INCLUDE_DIR}/spdlog/spdlog.h")
        target_include_directories(run_e2e PRIVATE ${SPDLOG_INCLUDE_DIR})
        if (FACTORLIB_ENABLE_TRACE_DEBUG)
            target_compile_definitions(run_e2e PRIVATE USE_SPDLOG=1 SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
        else()
            target_compile_definitions(run_e2e PRIVATE USE_SPDLOG=1 SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO FACTORLIB_NO_DEBUG_TRACE=1)
        endif()
    else()
        if (NOT FACTORLIB_ENABLE_TRACE_DEBUG)
            target_compile_definitions(run_e2e PRIVATE FACTORLIB_NO_DEBUG_TRACE=1)
        endif()
    endif()

    add_test(NAME run_e2e COMMAND run_e2e)
endif()
