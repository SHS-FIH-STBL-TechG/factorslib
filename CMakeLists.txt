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

# ---------- 第三方：Eigen ----------
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
if (EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
  message(STATUS "Eigen headers detected: ${EIGEN3_INCLUDE_DIR}")
  include_directories(${EIGEN3_INCLUDE_DIR})
else()
  message(WARNING "Eigen headers NOT found at ${EIGEN3_INCLUDE_DIR}")
  message(WARNING "Please download Eigen to third_party/eigen/")
  message(WARNING "Download from: https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz")
endif()

# ---------- 工程内公共头路径（也可逐目标设置） ----------
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../demo_header  # 添加 demo_header 路径
)

# ---------- 组件：工具库（日志 + 通用工具） ----------
add_library(factorlib_utils
        src/utils/log.cpp
        src/bridge/ingress.cpp
        src/utils/utils.cpp
        src/utils/data_adapter.cpp  # 添加数据适配器
)
target_include_directories(factorlib_utils PUBLIC include)

# ---------- 组件：基础因子库（现有因子 + 新因子） ----------
add_library(factor_basic
        src/basic_factors/tick_trans_orders.cpp
        src/gaussian_copula_factor.cpp  # 添加高斯 Copula 因子
)
target_link_libraries(factor_basic PUBLIC factorlib_utils)
target_include_directories(factor_basic PUBLIC include)

# ---------- （可选）spdlog：header-only 检测 ----------
# 把 spdlog 的 include/ 放到 third_party/spdlog/include/ 即可
set(SPDLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include)
if (EXISTS "${SPDLOG_INCLUDE_DIR}/spdlog/spdlog.h")
  message(STATUS "spdlog headers detected: ${SPDLOG_INCLUDE_DIR}")

  # 让日志在"编译期"不被裁剪（TRACE/DEBUG 也会编译）
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
        tests/gaussian_copula_factor_test.cpp
        tests/gtest_printer_zh.h  # 添加新因子测试
)

add_executable(run_tests ${FACTORLIB_TEST_SOURCES}
        tests/gtest_printer_zh.h)
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

# 为测试添加 demo_header 路径
target_include_directories(run_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../demo_header
)

add_test(NAME run_all COMMAND run_tests)

# ---------- 安装目标（可选） ----------
# 如果需要将库安装到系统，可以取消注释以下内容
# install(TARGETS factorlib_utils factor_basic
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib)
#
# install(DIRECTORY include/ DESTINATION include)
