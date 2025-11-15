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

# ---------- 工程内公共头路径 ----------
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
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

# ---------- spdlog：始终使用 third_party/spdlog ----------
set(SPDLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
message(STATUS "Using spdlog headers: ${SPDLOG_INCLUDE_DIR}")

# ========== 组件：工具库（日志 + 通用工具） ==========
add_library(factorlib_utils
        src/utils/log.cpp
        src/bridge/ingress.cpp
        src/utils/data_adapter.cpp
        src/utils/trading_time.cpp
        src/utils/nms_bucket_aggregator.cpp
        include/math/incremental_rank.h
        include/utils/config/feed_mode.h
        include/math/sliding_normal_eq.h
        # 注意：移除了 math_utils.cpp
)
target_include_directories(factorlib_utils PUBLIC
        ${FACTORLIB_THIRDPARTY_INCLUDES}
        ${SPDLOG_INCLUDE_DIR}
)

# spdlog/日志相关宏（utils）
if (FACTORLIB_ENABLE_TRACE_DEBUG)
    target_compile_definitions(factorlib_utils PUBLIC
            USE_SPDLOG=1
            SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
    )
else()
    target_compile_definitions(factorlib_utils PUBLIC
            USE_SPDLOG=1
            SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
            FACTORLIB_NO_DEBUG_TRACE=1
    )
endif()

# ---------- 组件：基础因子库（现有因子 + 新因子） ----------
add_library(factor_basic
        src/basic_factors/tick_trans_orders.cpp
        src/stat_factors/gaussian_copula_factor.cpp
        src/stat_factors/granger_causality_factor.cpp
        src/stat_factors/granger_causality_factor.h
        src/stat_factors/granger_causality_factor.cpp
        src/config/runtime_config.cpp
        src/stat_factors/least_squares_info_gain_factor.h
        src/stat_factors/least_squares_info_gain_factor.cpp
        src/stat_factors/probability_momentum_factor.cpp
)
target_link_libraries(factor_basic PUBLIC factorlib_utils)
target_include_directories(factor_basic PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${SPDLOG_INCLUDE_DIR}
)

# spdlog/日志相关宏（factor_basic）
if (FACTORLIB_ENABLE_TRACE_DEBUG)
    target_compile_definitions(factor_basic PUBLIC
            USE_SPDLOG=1
            SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
    )
else()
    target_compile_definitions(factor_basic PUBLIC
            USE_SPDLOG=1
            SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
            FACTORLIB_NO_DEBUG_TRACE=1
    )
endif()

# ---------- 测试 ----------
if (FACTORLIB_BUILD_TESTS AND FACTORLIB_BUILD_UNIT_TESTS)
    enable_testing()

    # 因子相关测试（统一 run_tests）
    set(FACTORLIB_TEST_SOURCES
            tests/basic_factors_tests/tick_trans_orders_test.cpp
            tests/test_wait.cpp
            tests/factor_compute_test.cpp
            tests/stat_factors_tests/gaussian_copula_factor_test.cpp
            tests/stat_factors_tests/granger_causality_factor_test.cpp
            tests/utils/test_config.h
            tests/utils/test_config.cpp
            tests/stat_factors_tests/least_squares_info_gain_factor_test.cpp
            tests/stat_factors_tests/probability_momentum_factor_test.cpp
    )

    add_executable(run_tests ${FACTORLIB_TEST_SOURCES} tests/gtest_printer_zh.h)
    target_link_libraries(run_tests PRIVATE factor_basic factorlib_utils gtest gtest_main)

    if (NOT MSVC)
        find_package(Threads REQUIRED)
        target_link_libraries(run_tests PRIVATE Threads::Threads)
    endif()

    # spdlog 配置（run_tests）
    target_include_directories(run_tests PRIVATE ${SPDLOG_INCLUDE_DIR})
    if (FACTORLIB_ENABLE_TRACE_DEBUG)
        target_compile_definitions(run_tests PRIVATE
                USE_SPDLOG=1
                SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
        )
    else()
        target_compile_definitions(run_tests PRIVATE
                USE_SPDLOG=1
                SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
                FACTORLIB_NO_DEBUG_TRACE=1
        )
    endif()

    # demo_header 路径
    target_include_directories(run_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../demo_header
    )

    add_test(NAME run_all COMMAND run_tests)

    # ---------- 数学工具单独测试目标 ----------
    add_executable(math_tests
            tests/math_tests/all_math_tests.cpp
            tests/math_tests/distributions_test.cpp
            tests/math_tests/incremental_rank_test.cpp
            tests/math_tests/linear_algebra_test.cpp
            tests/math_tests/numeric_utils_test.cpp
            tests/math_tests/sliding_normal_eq_test.cpp
            "tests/math_tests/ sliding_statistics_test.cpp .cpp"
            tests/math_tests/statistics_test.cpp
    )
    target_link_libraries(math_tests PRIVATE factorlib_utils gtest gtest_main)

    if (NOT MSVC)
        find_package(Threads REQUIRED)
        target_link_libraries(math_tests PRIVATE Threads::Threads)
    endif()

    target_include_directories(math_tests PRIVATE ${SPDLOG_INCLUDE_DIR})
    if (FACTORLIB_ENABLE_TRACE_DEBUG)
        target_compile_definitions(math_tests PRIVATE
                USE_SPDLOG=1
                SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
        )
    else()
        target_compile_definitions(math_tests PRIVATE
                USE_SPDLOG=1
                SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
                FACTORLIB_NO_DEBUG_TRACE=1
        )
    endif()

    add_test(NAME math_tests COMMAND math_tests)
endif()

# ---------- E2E 测试 ----------
if (FACTORLIB_WITH_DEMO_E2E)
    enable_testing()

    add_executable(run_e2e tests/integration/demo_min_e2e_test.cpp)
    target_link_libraries(run_e2e PRIVATE factor_basic factorlib_utils gtest gtest_main)

    if (NOT MSVC)
        find_package(Threads REQUIRED)
        target_link_libraries(run_e2e PRIVATE Threads::Threads)
    endif()

    # demo_header：优先用 INTERFACE 目标，没有则直接 include 路径
    if (TARGET demo_headers)
        target_link_libraries(run_e2e PRIVATE demo_headers)
    else()
        target_include_directories(run_e2e PRIVATE
                ${CMAKE_CURRENT_LIST_DIR}/../demo_header
        )
    endif()

    # spdlog 配置（run_e2e）
    target_include_directories(run_e2e PRIVATE ${SPDLOG_INCLUDE_DIR})
    if (FACTORLIB_ENABLE_TRACE_DEBUG)
        target_compile_definitions(run_e2e PRIVATE
                USE_SPDLOG=1
                SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
        )
    else()
        target_compile_definitions(run_e2e PRIVATE
                USE_SPDLOG=1
                SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
                FACTORLIB_NO_DEBUG_TRACE=1
        )
    endif()

    add_test(NAME run_e2e COMMAND run_e2e)
endif()
