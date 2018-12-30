cmake_minimum_required(VERSION 3.0.0)
project(KFilter VERSION 0.1.0)

include(CTest)
enable_testing()

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_library(KFilter KFilter.cpp)
target_link_libraries(KFilter Eigen3::Eigen)

add_executable(test_filter test_filter.cpp)
target_link_libraries(test_filter KFilter)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


