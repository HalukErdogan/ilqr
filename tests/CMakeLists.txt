find_package(GTest CONFIG REQUIRED)

# Test numerical diff of dynamic system
add_executable(system_test ilqr/system/test_discrete_system_with_finite_diff.cc)
target_link_libraries(system_test PRIVATE ilqr GTest::gtest_main)

# Test numerical diff of cost function
add_executable(cost_function_test ilqr/cost_function/test_cost_function_with_finite_diff.cc)
target_link_libraries(cost_function_test PRIVATE ilqr GTest::gtest_main)

# Include GoogleTest for test discovery
include(GoogleTest)
gtest_discover_tests(system_test)
gtest_discover_tests(cost_function_test)