
# 顶层 CMake 配置：生成 factorlib 库与测试可执行文件
cmake_minimum_required(VERSION 3.15)

# 工程名与 C++ 标准
project(factorlib LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 关闭不必要的警告（可按需调整）
if (MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else()
  add_compile_options(-Wall -Wextra -O2)
endif()

# 第三方：googletest（你需要把 googletest 的 include/ 与 src/ 放入 third_party/googletest/googletest/）
add_subdirectory(third_party/googletest)

# 头文件路径
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
)
target_link_libraries(factor_basic PUBLIC factorlib_utils)
target_include_directories(factor_basic PUBLIC include)

# ---------- 测试 ----------
enable_testing()

# 测试源文件
set(FACTORLIB_TEST_SOURCES
  tests/test_bus_and_factor.cpp
  tests/test_wait.cpp
)

add_executable(run_tests ${FACTORLIB_TEST_SOURCES})
target_link_libraries(run_tests PRIVATE
  factor_basic
  factorlib_utils
  gtest
  gtest_main
  pthread
)

add_test(NAME run_all COMMAND run_tests)
