find_package(GTest REQUIRED)
include(GoogleTest)

# Checks that every dependency in the container compiles, links and runs.
add_executable(dependencies_test dependencies_test.cc)
target_link_libraries(dependencies_test PRIVATE mini_vio_deps GTest::gtest_main)
gtest_discover_tests(dependencies_test)
