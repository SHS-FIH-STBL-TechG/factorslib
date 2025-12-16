# tests/Kline_factors_tests/CMakeLists.txt
# 默认相对路径基于 tests/Kline_factors_tests/

add_executable(kline_factors_tests
        kline_factors_tests_main.cpp
        ar1_return_factor_test.cpp
        gls_drift_z_factor_test.cpp
        hmm2_logodds_factor_test.cpp
        bayes_drift_logodds_factor_test.cpp
        high_volume_remaining_factor_test.cpp
        low_freq_return_test.cpp
)

target_link_libraries(kline_factors_tests PRIVATE
        factor_basic
        factorlib_utils
        gtest
        gtest_main
)

target_include_directories(kline_factors_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

if (NOT MSVC)
    target_link_libraries(kline_factors_tests PRIVATE Threads::Threads)
endif()

add_test(NAME kline_factors_tests COMMAND kline_factors_tests)
