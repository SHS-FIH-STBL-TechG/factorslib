# tests/stat_factors_tests/CMakeLists.txt
# 该目录下的 .cpp 文件默认是相对路径：等价于 tests/stat_factors_tests/xxx.cpp

# stat_factors 专用测试目标
add_executable(stat_factors_tests
        gaussian_copula_factor_test.cpp
        granger_causality_factor_test.cpp
        least_squares_info_gain_factor_test.cpp
        probability_momentum_factor_test.cpp
        ../utils/test_config.h
        ../utils/test_config.cpp
)

target_link_libraries(stat_factors_tests PRIVATE
        factor_basic        # stat_factors 依赖 factor_basic
        factorlib_utils
        gtest
        gtest_main
)

# 非 MSVC 平台链接线程库（pthread 等）
if (NOT MSVC)
    target_link_libraries(stat_factors_tests PRIVATE Threads::Threads)
endif()

# 注册为 CTest 测试
add_test(NAME stat_factors_tests COMMAND stat_factors_tests)
