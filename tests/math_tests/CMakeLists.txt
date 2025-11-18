# tests/math_tests/CMakeLists.txt
# 该目录下的 .cpp 文件默认是相对路径：等价于 tests/math_tests/xxx.cpp

# 数学工具单独测试目标
add_executable(math_tests
        all_math_tests.cpp
        sliding_statistics_test.cpp
        incremental_rank_test.cpp
        sliding_normal_eq_test.cpp
        linear_algebra_test.cpp
        numeric_utils_test.cpp
        statistics_test.cpp
        distributions_test.cpp
        fractional_diff_test.cpp
        rolling_autocorr_test.cpp
        rolling_entropy_test.cpp
        modwt_test.cpp
        symbolic_dynamics_test.cpp
        online_pca_test.cpp
        vector_field_analysis_test.cpp
        memory_kernel_test.cpp
        hawkes_intensity_test.cpp
        tda_rips_h1_test.cpp
        fnn_test.cpp
        info_geometry_test.cpp
        persistent_h0_test.cpp
        rosenstein_lyapunov_test.cpp
)

target_link_libraries(math_tests PRIVATE
        factorlib_utils
        gtest
        gtest_main
)

# 非 MSVC 平台链接线程库（pthread 等）
if (NOT MSVC)
    target_link_libraries(math_tests PRIVATE Threads::Threads)
endif()

# 注册为 CTest 测试
add_test(NAME math_tests COMMAND math_tests)
