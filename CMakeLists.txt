# 顶层 CMake 配置：生成 factorlib 库与测试可执行文件
cmake_minimum_required(VERSION 3.10)

project(factorlib LANGUAGES CXX)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(FACTORLIB_ON_WINDOWS_MINGW OFF)
if (WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(FACTORLIB_ON_WINDOWS_MINGW ON)
endif()

# ========================
# 选项
# ========================
option(FACTORLIB_BUILD_TESTS "Build tests" ON)
option(FACTORLIB_USE_THIRD_PARTY "Prefer bundled third_party libs" ON)
option(FACTORLIB_ENABLE_TRACE_DEBUG "Compile TRACE/DEBUG logs" OFF)
option(FACTORLIB_BUILD_UNIT_TESTS "Build factorlib unit tests " ON)
option(FACTORLIB_BUILD_MATH_TESTS "Build math_tests unit tests" OFF)
option(FACTORLIB_ENABLE_PARALLEL_INGRESS "Dispatch ingress events to factors in parallel" OFF)
option(FACTORLIB_ENABLE_IC_RUNTIME "Enable IC evaluation runtime" ON)
set(_factorlib_perfetto_default ON)
if (FACTORLIB_ON_WINDOWS_MINGW)
    set(_factorlib_perfetto_default OFF)
endif()
option(FACTORLIB_ENABLE_PERFETTO "Enable Perfetto performance tracing" ${_factorlib_perfetto_default})
set(_factorlib_branch_coverage_default OFF)
if (FACTORLIB_ON_WINDOWS_MINGW)
    set(_factorlib_branch_coverage_default ON)
endif()
option(FACTORLIB_ENABLE_BRANCH_COVERAGE "Collect branch coverage via gcov after running tests" ${_factorlib_branch_coverage_default})
# 运行在 Windows + MinGW 时，强制关闭 Perfetto（SDK 不支持该组合）
if (FACTORLIB_ON_WINDOWS_MINGW AND FACTORLIB_ENABLE_PERFETTO)
    message(WARNING "Disabling Perfetto tracing: Perfetto is not supported on Windows + MinGW toolchains.")
    set(FACTORLIB_ENABLE_PERFETTO OFF CACHE BOOL "Enable Perfetto performance tracing" FORCE)
endif()
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
        FACTORLIB_ENABLE_IC_RUNTIME
        FACTORLIB_ENABLE_PERFETTO
        FACTORLIB_ENABLE_BRANCH_COVERAGE
)
    message(STATUS "  ${opt} = ${${opt}}")
endforeach()

message(STATUS "============================================")

# ========================
# 路径重定位：third_party / RollWinOp 必须位于仓库外部
# ========================
# 只在用户没有手动设置时，才按平台给默认值
if (NOT DEFINED FACTORLIB_THIRD_PARTY_DIR)
    set(_factorlib_third_party_guess "${CMAKE_CURRENT_SOURCE_DIR}/../third_party")
    if (DEFINED ENV{FACTORLIB_THIRD_PARTY_DIR} AND NOT "$ENV{FACTORLIB_THIRD_PARTY_DIR}" STREQUAL "")
        set(_factorlib_third_party_guess "$ENV{FACTORLIB_THIRD_PARTY_DIR}")
    elseif(UNIX AND EXISTS "/mnt/disk5/per/users/guqk/v_1.0.0/third_party")
        set(_factorlib_third_party_guess "/mnt/disk5/per/users/guqk/v_1.0.0/third_party")
    endif()
    set(FACTORLIB_THIRD_PARTY_DIR "${_factorlib_third_party_guess}" CACHE PATH "Path to external third_party directory" FORCE)
else()
    set(FACTORLIB_THIRD_PARTY_DIR "${FACTORLIB_THIRD_PARTY_DIR}" CACHE PATH "Path to external third_party directory" FORCE)
endif()

if (NOT DEFINED ROLLWINOP_ROOT)
    set(_factorlib_rollwinop_guess "${CMAKE_CURRENT_SOURCE_DIR}/../RollWinOp")
    if (DEFINED ENV{ROLLWINOP_ROOT} AND NOT "$ENV{ROLLWINOP_ROOT}" STREQUAL "")
        set(_factorlib_rollwinop_guess "$ENV{ROLLWINOP_ROOT}")
    elseif(UNIX AND EXISTS "/mnt/disk5/per/users/guqk/v_1.0.0/RollWinOp")
        set(_factorlib_rollwinop_guess "/mnt/disk5/per/users/guqk/v_1.0.0/RollWinOp")
    endif()
    set(ROLLWINOP_ROOT "${_factorlib_rollwinop_guess}" CACHE PATH "Path to RollWinOp checkout" FORCE)
else()
    set(ROLLWINOP_ROOT "${ROLLWINOP_ROOT}" CACHE PATH "Path to RollWinOp checkout" FORCE)
endif()

if (NOT EXISTS "${FACTORLIB_THIRD_PARTY_DIR}")
    message(FATAL_ERROR "third_party directory not found at ${FACTORLIB_THIRD_PARTY_DIR}.
    请先将 third_party 整个目录移动到仓库外的 ../third_party。")
endif()
message(STATUS "Using third_party directory: ${FACTORLIB_THIRD_PARTY_DIR}")

if (NOT EXISTS "${ROLLWINOP_ROOT}/include/math")
    message(FATAL_ERROR "RollWinOp math headers not found at ${ROLLWINOP_ROOT}/include/math.
    请先将 include/math 目录整体移动到 ../RollWinOp/include/math。")
endif()
set(FACTORLIB_ROLLWINOP_INCLUDE_DIR ${ROLLWINOP_ROOT}/include)
set(FACTORLIB_MATH_HEADER_DIR ${ROLLWINOP_ROOT}/include/math)
set(FACTORLIB_MATH_TESTS_SOURCE_DIR ${ROLLWINOP_ROOT}/tests/math_tests)
message(STATUS "Using external RollWinOp includes: ${FACTORLIB_ROLLWINOP_INCLUDE_DIR}")

# 继承 RollWinOp 的 INTERFACE 目标（仅使用头文件，不编译其测试）
set(ROLLWINOP_BUILD_TESTS OFF CACHE BOOL "Disable RollWinOp tests when embedded" FORCE)
set(ROLLWINOP_ENABLE_TRACE_DEBUG ${FACTORLIB_ENABLE_TRACE_DEBUG} CACHE BOOL "" FORCE)
add_subdirectory(${ROLLWINOP_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/RollWinOp)

# ========================
# 编译选项
# ========================
if (MSVC)
    add_compile_options(/utf-8)  # Ensure source is treated as UTF-8 on MSVC
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_compile_options(
            /Zc:__cplusplus       # 让 __cplusplus 正确报告 C++17
            /d2ReducedOptimizeHugeFunctions  # 避免 MSVC 在巨型模板上触发 C1001
            /d2SSAOptimizer-      # 关闭 SSA 优化，缓解编译器 BUG
    )
else()
    add_compile_options(-Wall -Wextra -O2)
endif()

# GCC 7/8 仍需单独链接 stdc++fs 才能使用 std::filesystem
set(FACTORLIB_STD_FS_LIB "")
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    set(FACTORLIB_STD_FS_LIB stdc++fs)
endif()

if (FACTORLIB_ENABLE_BRANCH_COVERAGE)
    if (MSVC)
        message(FATAL_ERROR "FACTORLIB_ENABLE_BRANCH_COVERAGE requires a GCC/Clang-like toolchain (not MSVC).")
    endif()
    add_compile_options(--coverage)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
    find_program(FACTORLIB_GCOV_PATH NAMES gcov gcov.exe)
    if (NOT FACTORLIB_GCOV_PATH)
        message(FATAL_ERROR "gcov not found but FACTORLIB_ENABLE_BRANCH_COVERAGE is ON.")
    endif()
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
    set(FACTORLIB_GTEST_SOURCE_DIR ${FACTORLIB_THIRD_PARTY_DIR}/googletest)
    if (EXISTS "${FACTORLIB_GTEST_SOURCE_DIR}/CMakeLists.txt")
        add_subdirectory(${FACTORLIB_GTEST_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/third_party/googletest)
    else()
        message(FATAL_ERROR "googletest not found at ${FACTORLIB_GTEST_SOURCE_DIR}")
    endif()
endif()

# ---------- 第三方：Eigen ----------
if (FACTORLIB_USE_THIRD_PARTY)
    set(EIGEN3_INCLUDE_DIR ${FACTORLIB_THIRD_PARTY_DIR}/eigen)
    if (EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
        message(STATUS "Using bundled Eigen: ${EIGEN3_INCLUDE_DIR}")
        # 全局 Eigen 头文件
        include_directories(${EIGEN3_INCLUDE_DIR})
    else()
        message(WARNING "Eigen headers NOT found at ${EIGEN3_INCLUDE_DIR}")
        message(WARNING "Please download Eigen to ${FACTORLIB_THIRD_PARTY_DIR}/eigen/")
        message(WARNING "Download from: https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz")
    endif()
endif()

# ---------- 工程内公共头路径（全局可用） ----------
set(FACTORLIB_PUBLIC_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
if (DEFINED FACTORLIB_ROLLWINOP_INCLUDE_DIR)
    list(APPEND FACTORLIB_PUBLIC_INCLUDE_DIRS ${FACTORLIB_ROLLWINOP_INCLUDE_DIR})
endif()
include_directories(${FACTORLIB_PUBLIC_INCLUDE_DIRS})

# ====== Boost（使用 bcp 导出的最小头，避免 FindBoost / CMP0167） ======
set(BOOST_VENDOR_DIR ${FACTORLIB_THIRD_PARTY_DIR}/boost)
if (EXISTS "${BOOST_VENDOR_DIR}/boost/config.hpp")
    message(STATUS "Using bundled Boost headers (bcp): ${BOOST_VENDOR_DIR}")
    # 全局 Boost 头文件
    include_directories(${BOOST_VENDOR_DIR})
else()
    message(FATAL_ERROR "Bundled Boost headers not found at ${BOOST_VENDOR_DIR}.
    请先用 bcp 导出到 ${FACTORLIB_THIRD_PARTY_DIR}/boost，再重新配置。")
endif()

# ---------- nanoflann ----------
set(NANOFLANN_INCLUDE_DIR ${FACTORLIB_THIRD_PARTY_DIR}/nanoflann)
if (EXISTS "${NANOFLANN_INCLUDE_DIR}/nanoflann.hpp")
    message(STATUS "Using bundled nanoflann: ${NANOFLANN_INCLUDE_DIR}")
    # 全局 nanoflann 头文件
    include_directories(${NANOFLANN_INCLUDE_DIR})
else()
    message(FATAL_ERROR "nanoflann.hpp not found at ${NANOFLANN_INCLUDE_DIR}.
    请把 nanoflann.hpp 放到 ${FACTORLIB_THIRD_PARTY_DIR}/nanoflann 目录下。")
endif()

# ---------- spdlog：始终使用 third_party/spdlog ----------
set(SPDLOG_INCLUDE_DIR ${FACTORLIB_THIRD_PARTY_DIR}/spdlog/include)
message(STATUS "Using spdlog headers: ${SPDLOG_INCLUDE_DIR}")
# 全局 spdlog 头文件
include_directories(${SPDLOG_INCLUDE_DIR})

# ---------- 全局日志相关宏（所有 target 共用） ----------
if (FACTORLIB_ENABLE_TRACE_DEBUG)
    add_definitions(
            -DUSE_SPDLOG=1
            -DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
            -DFMT_USE_CONSTEXPR=0
            -DFMT_USE_CONSTEXPR_STRING=0
    )
else()
    add_definitions(
            -DUSE_SPDLOG=1
            -DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
            -DFACTORLIB_NO_DEBUG_TRACE=1
            -DFMT_USE_CONSTEXPR=0
            -DFMT_USE_CONSTEXPR_STRING=0
    )
endif()

if (FACTORLIB_ENABLE_PARALLEL_INGRESS)
    add_definitions(-DFACTORLIB_ENABLE_PARALLEL_INGRESS=1)
endif()

if (FACTORLIB_ENABLE_IC_RUNTIME)
    add_definitions(-DFACTORLIB_ENABLE_IC_RUNTIME=1)
else()
    add_definitions(-DFACTORLIB_ENABLE_IC_RUNTIME=0)
endif()

if (FACTORLIB_ENABLE_PERFETTO)
    add_definitions(-DFACTORLIB_ENABLE_PERFETTO=1)
endif()

# ====== Perfetto SDK（如果启用） ======
if (FACTORLIB_ENABLE_PERFETTO)
    set(PERFETTO_SDK_DIR ${FACTORLIB_THIRD_PARTY_DIR}/perfetto/sdk)
    if (EXISTS "${PERFETTO_SDK_DIR}/perfetto.h" AND EXISTS "${PERFETTO_SDK_DIR}/perfetto.cc")
        message(STATUS "Using Perfetto SDK: ${PERFETTO_SDK_DIR}")
        # 创建 Perfetto 库
        add_library(perfetto STATIC ${PERFETTO_SDK_DIR}/perfetto.cc)
        target_include_directories(perfetto PUBLIC ${PERFETTO_SDK_DIR})
        target_compile_features(perfetto PUBLIC cxx_std_17)

        # Windows 需要特殊配置
        if (MSVC)
            target_compile_options(perfetto PRIVATE /bigobj)
            target_compile_definitions(perfetto PUBLIC WIN32_LEAN_AND_MEAN NOMINMAX)
            target_link_libraries(perfetto PUBLIC ws2_32)
        else()
            target_link_libraries(perfetto PUBLIC ${CMAKE_THREAD_LIBS_INIT})
        endif()
    else()
        message(WARNING "Perfetto SDK not found at ${PERFETTO_SDK_DIR}")
        message(WARNING "Please download Perfetto SDK amalgamated files:")
        message(WARNING "  perfetto.h and perfetto.cc")
        message(WARNING "from: https://github.com/google/perfetto/releases")
        message(WARNING "Place them in: ${FACTORLIB_THIRD_PARTY_DIR}/perfetto/sdk/")
        message(FATAL_ERROR "Cannot enable Perfetto without SDK files")
    endif()
endif()

set(FACTORLIB_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
if (DEFINED FACTORLIB_ROLLWINOP_INCLUDE_DIR)
    list(APPEND FACTORLIB_INTERFACE_INCLUDE_DIRS ${FACTORLIB_ROLLWINOP_INCLUDE_DIR})
endif()

set(FACTORLIB_MATH_INCREMENTAL_RANK_HEADER ${FACTORLIB_MATH_HEADER_DIR}/incremental_rank.h)
set(FACTORLIB_MATH_SLIDING_NORMAL_EQ_HEADER ${FACTORLIB_MATH_HEADER_DIR}/sliding_normal_eq.h)

# ========== 组件：工具库（日志 + 通用工具） ==========
add_library(factorlib_utils
        src/utils/log.cpp
        src/bridge/ingress.cpp
        src/utils/data_adapter.cpp
        src/utils/trading_time.cpp
        src/utils/nms_bucket_aggregator.cpp
        src/utils/trace_helper.cpp
        src/tools/factor_ic_runtime.cpp
        ${FACTORLIB_MATH_INCREMENTAL_RANK_HEADER}
        include/config/feed_mode.h
        ${FACTORLIB_MATH_SLIDING_NORMAL_EQ_HEADER}
        tests/utils/NumCountSimulation.cpp
)

# 对外暴露工程内头文件（第三方已经通过全局 include_directories 提供）
target_include_directories(factorlib_utils PUBLIC
        ${FACTORLIB_INTERFACE_INCLUDE_DIRS}
)

if (MSVC)
    target_compile_definitions(factorlib_utils
            INTERFACE _USE_MATH_DEFINES
    )
endif()

# 链接 Perfetto 库（如果启用）
if (FACTORLIB_ENABLE_PERFETTO)
    target_link_libraries(factorlib_utils PUBLIC perfetto)
endif()
target_link_libraries(factorlib_utils PUBLIC rollwinop)
if (FACTORLIB_STD_FS_LIB)
    target_link_libraries(factorlib_utils PUBLIC ${FACTORLIB_STD_FS_LIB})
endif()

# ---------- 组件：基础因子库（现有因子 + 新因子） ----------
add_library(factor_basic
        src/factors/basic/tick_trans_orders.cpp
        src/factors/stat/gaussian_copula_factor.cpp
        src/factors/stat/granger_causality_factor.cpp
        src/config/runtime_config.cpp
        src/factors/stat/least_squares_info_gain_factor.cpp
        src/factors/stat/probability_momentum_factor.cpp
        src/factors/stat/wavelet_trend_energy_factor.cpp
        src/factors/stat/memory_kernel_decay_factor.cpp
        src/factors/stat/volume_multiscale_autocorr_factor.cpp
        src/factors/stat/volume_mgf_factor.cpp
        src/factors/stat/hawkes_intensity_factor.cpp
        src/factors/stat/symbolic_transition_factor.cpp
        src/factors/stat/pca_structure_stability_factor.cpp
        src/factors/stat/pca_angular_momentum_factor.cpp
        src/factor_factory.cpp
        include/factors/stat/gaussian_copula_factor.h
        include/factors/stat/granger_causality_factor.h
        include/factors/stat/least_squares_info_gain_factor.h
        include/factors/stat/probability_momentum_factor.h
        include/factors/stat/wavelet_trend_energy_factor.h
        include/factors/stat/memory_kernel_decay_factor.h
        include/factors/stat/volume_multiscale_autocorr_factor.h
        include/factors/stat/volume_mgf_factor.h
        include/factors/stat/hawkes_intensity_factor.h
        include/factors/stat/symbolic_transition_factor.h
        include/factors/stat/pca_structure_stability_factor.h
        include/factors/stat/pca_angular_momentum_factor.h
        include/factors/basic/tick_trans_orders.h
)

target_link_libraries(factor_basic PUBLIC factorlib_utils)
target_include_directories(factor_basic PUBLIC
        ${FACTORLIB_INTERFACE_INCLUDE_DIRS}
)

# ---------- 单元测试 ----------
if (FACTORLIB_BUILD_TESTS AND FACTORLIB_BUILD_UNIT_TESTS)
    enable_testing()

    # 因子相关测试（这里只保留非 stat_factors 的，stat_factors 单独拆目录）
    set(FACTORLIB_TEST_SOURCES
            tests/basic_factors_tests/tick_trans_orders_test.cpp
            tests/test_wait.cpp
            tests/utils/databus_test.cpp
            tests/utils/trace_helper_test.cpp
            tests/factor_compute_test.cpp
            tests/utils/test_config.h
            tests/utils/test_config.cpp
            tests/utils/NumCountSimulation.cpp
            tests/utils/NumCountSimulation.h
            tests/tools/factor_ic_runtime_test.cpp
    )

    add_executable(factorlib_unit_tests ${FACTORLIB_TEST_SOURCES} tests/gtest_printer_zh.h)
    # Keep the binary name as run_tests.exe but avoid clashing with Visual Studio's RUN_TESTS target
    set_target_properties(factorlib_unit_tests PROPERTIES OUTPUT_NAME run_tests)
    target_link_libraries(factorlib_unit_tests PRIVATE factor_basic factorlib_utils gtest gtest_main)

    if (NOT MSVC)
        target_link_libraries(factorlib_unit_tests PRIVATE Threads::Threads)
    endif()

    # demo_header 路径
    target_include_directories(factorlib_unit_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../demo_header
    )

    add_test(NAME run_all COMMAND $<TARGET_FILE:factorlib_unit_tests>)

    # stat_factors 测试子目录
    add_subdirectory(tests/stat_factors_tests)

    # 数学工具测试子目录
    if (FACTORLIB_BUILD_MATH_TESTS)
        add_subdirectory(${FACTORLIB_MATH_TESTS_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/math_tests)
    endif()

    add_executable(factor_ic_tool
            tests/factor_ic_tool.cpp
    )
    target_link_libraries(factor_ic_tool PRIVATE factor_basic factorlib_utils)

    if (FACTORLIB_ENABLE_BRANCH_COVERAGE)
        set(_factorlib_branch_cov_deps run_all stat_factors_tests)
        set(FACTORLIB_BRANCH_COVERAGE_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/cmake/RunBranchCoverage.cmake)
        set(FACTORLIB_BRANCH_COVERAGE_REPORT ${CMAKE_BINARY_DIR}/branch_coverage.txt)
        add_test(NAME branch_coverage
                COMMAND ${CMAKE_COMMAND}
                -DGCOV_PATH=${FACTORLIB_GCOV_PATH}
                -DBINARY_DIR=${CMAKE_BINARY_DIR}
                -DREPORT_PATH=${FACTORLIB_BRANCH_COVERAGE_REPORT}
                -P ${FACTORLIB_BRANCH_COVERAGE_SCRIPT})
        set_tests_properties(branch_coverage PROPERTIES DEPENDS "${_factorlib_branch_cov_deps}")
    endif()
endif()
