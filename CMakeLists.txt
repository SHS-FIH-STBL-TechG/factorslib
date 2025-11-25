# 顶层 CMake 配置：生成 factorlib 库与测试可执行文件
cmake_minimum_required(VERSION 3.10)

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
option(FACTORLIB_BUILD_UNIT_TESTS "Build factorlib unit tests " ON)
option(FACTORLIB_BUILD_MATH_TESTS "Build math_tests unit tests" OFF)
option(FACTORLIB_ENABLE_PARALLEL_INGRESS "Dispatch ingress events to factors in parallel" OFF)
# 打印所有相关选项
message(STATUS "========== factorlib CMake options ==========")

foreach(opt
        FACTORLIB_BUILD_TESTS
        FACTORLIB_USE_THIRD_PARTY
        FACTORLIB_ENABLE_TRACE_DEBUG
        FACTORLIB_WITH_DEMO_E2E
        FACTORLIB_BUILD_UNIT_TESTS
        FACTORLIB_BUILD_MATH_TESTS
        FACTORLIB_ENABLE_PARALLEL_INGRESS
)
    message(STATUS "  ${opt} = ${${opt}}")
endforeach()

message(STATUS "============================================")

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
# 全局线程库（非 MSVC）
# ========================
if (NOT MSVC)
    find_package(Threads REQUIRED)
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
        # 全局 Eigen 头文件
        include_directories(${EIGEN3_INCLUDE_DIR})
    else()
        message(WARNING "Eigen headers NOT found at ${EIGEN3_INCLUDE_DIR}")
        message(WARNING "Please download Eigen to third_party/eigen/")
        message(WARNING "Download from: https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz")
    endif()
endif()

# ---------- 工程内公共头路径（全局可用） ----------
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ====== Boost（使用 bcp 导出的最小头，避免 FindBoost / CMP0167） ======
set(BOOST_VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost)
if (EXISTS "${BOOST_VENDOR_DIR}/boost/config.hpp")
    message(STATUS "Using bundled Boost headers (bcp): ${BOOST_VENDOR_DIR}")
    # 全局 Boost 头文件
    include_directories(${BOOST_VENDOR_DIR})
else()
    message(FATAL_ERROR "Bundled Boost headers not found at ${BOOST_VENDOR_DIR}.
    请先用 bcp 导出到 factors_lib/third_party/boost，再重新配置。")
endif()

# ---------- nanoflann ----------
set(NANOFLANN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanoflann)
if (EXISTS "${NANOFLANN_INCLUDE_DIR}/nanoflann.hpp")
    message(STATUS "Using bundled nanoflann: ${NANOFLANN_INCLUDE_DIR}")
    # 全局 nanoflann 头文件
    include_directories(${NANOFLANN_INCLUDE_DIR})
else()
    message(FATAL_ERROR "nanoflann.hpp not found at ${NANOFLANN_INCLUDE_DIR}.
    请把 nanoflann.hpp 放到 factorlib/third_party/nanoflann 目录下。")
endif()

# ---------- spdlog：始终使用 third_party/spdlog ----------
set(SPDLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
message(STATUS "Using spdlog headers: ${SPDLOG_INCLUDE_DIR}")
# 全局 spdlog 头文件
include_directories(${SPDLOG_INCLUDE_DIR})

# ---------- 全局日志相关宏（所有 target 共用） ----------
if (FACTORLIB_ENABLE_TRACE_DEBUG)
    add_definitions(
            -DUSE_SPDLOG=1
            -DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
    )
else()
    add_definitions(
            -DUSE_SPDLOG=1
            -DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
            -DFACTORLIB_NO_DEBUG_TRACE=1
    )
endif()

if (FACTORLIB_ENABLE_PARALLEL_INGRESS)
    add_definitions(-DFACTORLIB_ENABLE_PARALLEL_INGRESS=1)
endif()

# ========== 组件：工具库（日志 + 通用工具） ==========
add_library(factorlib_utils
        src/utils/log.cpp
        src/bridge/ingress.cpp
        src/utils/data_adapter.cpp
        src/utils/trading_time.cpp
        src/utils/nms_bucket_aggregator.cpp
        src/utils/processing_axes.cpp
        include/math/incremental_rank.h
        include/utils/config/feed_mode.h
        include/math/sliding_normal_eq.h
        tests/utils/NumCountSimulation.cpp
)

# 对外暴露工程内头文件（第三方已经通过全局 include_directories 提供）
target_include_directories(factorlib_utils PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (MSVC)
    target_compile_definitions(factorlib_utils
            INTERFACE _USE_MATH_DEFINES
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
        src/stat_factors/least_squares_info_gain_factor.cpp
        src/stat_factors/probability_momentum_factor.cpp
        src/stat_factors/wavelet_trend_energy_factor.cpp
        src/stat_factors/memory_kernel_decay_factor.cpp
        src/stat_factors/volume_multiscale_autocorr_factor.cpp
        src/stat_factors/volume_mgf_factor.cpp
        src/stat_factors/hawkes_intensity_factor.cpp
        src/stat_factors/symbolic_transition_factor.cpp
        src/stat_factors/pca_structure_stability_factor.cpp
        src/stat_factors/pca_angular_momentum_factor.cpp
        src/factor_factory.cpp
)

target_link_libraries(factor_basic PUBLIC factorlib_utils)
target_include_directories(factor_basic PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---------- 单元测试 ----------
if (FACTORLIB_BUILD_TESTS AND FACTORLIB_BUILD_UNIT_TESTS)
    enable_testing()

    # 因子相关测试（这里只保留非 stat_factors 的，stat_factors 单独拆目录）
    set(FACTORLIB_TEST_SOURCES
            tests/basic_factors_tests/tick_trans_orders_test.cpp
            tests/test_wait.cpp
            tests/factor_compute_test.cpp
            tests/utils/test_config.h
            tests/utils/test_config.cpp
            tests/utils/NumCountSimulation.cpp
            tests/utils/NumCountSimulation.h
    )

    add_executable(run_tests ${FACTORLIB_TEST_SOURCES} tests/gtest_printer_zh.h)
    target_link_libraries(run_tests PRIVATE factor_basic factorlib_utils gtest gtest_main)

    if (NOT MSVC)
        target_link_libraries(run_tests PRIVATE Threads::Threads)
    endif()

    # demo_header 路径
    target_include_directories(run_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../demo_header
    )

    add_test(NAME run_all COMMAND run_tests)

    # stat_factors 测试子目录
    add_subdirectory(tests/stat_factors_tests)

    # 数学工具测试子目录
    if (FACTORLIB_BUILD_MATH_TESTS)
        add_subdirectory(tests/math_tests)
    endif()

    add_executable(factor_ic_tool
            tests/factor_ic_tool.cpp
    )
endif()
