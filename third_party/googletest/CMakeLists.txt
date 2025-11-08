
# 最小 CMake：直接从 googletest 源码构建 gtest 与 gtest_main
# 你需要把官方源码放到：third_party/googletest/googletest/
# 目录结构必须包含：googletest/include 与 googletest/src

cmake_minimum_required(VERSION 3.15)

# 包含目录
set(GTEST_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/googletest)
set(GTEST_INCLUDE_DIR ${GTEST_ROOT}/include)
set(GTEST_SRC_DIR ${GTEST_ROOT}/src)

# 生成静态库 gtest
add_library(gtest ${GTEST_SRC_DIR}/gtest-all.cc)
target_include_directories(gtest PUBLIC ${GTEST_INCLUDE_DIR} ${GTEST_ROOT})
if (MSVC)
  target_compile_definitions(gtest PRIVATE -DGTEST_HAS_PTHREAD=0)
else()
  if (MSVC)
  target_compile_definitions(gtest PRIVATE -DGTEST_HAS_PTHREAD=0)
else()
  target_compile_definitions(gtest PRIVATE -DGTEST_HAS_PTHREAD=1)
endif()
endif()

# 生成静态库 gtest_main（包含 main 函数）
add_library(gtest_main ${GTEST_SRC_DIR}/gtest_main.cc)
target_link_libraries(gtest_main PUBLIC gtest)
target_include_directories(gtest_main PUBLIC ${GTEST_INCLUDE_DIR} ${GTEST_ROOT})
