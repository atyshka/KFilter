cmake_minimum_required(VERSION 3.0)
project(KFilter VERSION 0.1.0)

include(CTest)
enable_testing()

find_package (Eigen3 REQUIRED NO_MODULE)

if (TARGET Eigen3::Eigen)
  add_compile_definitions(USE_NEW_EIGEN_HEADERS)

  add_library(KFilter src/KFilter.cpp)
  target_include_directories(KFilter PUBLIC include)
  target_link_libraries(KFilter Eigen3::Eigen)

  add_executable(test_filter test_filter.cpp)
  target_link_libraries(test_filter KFilter)
else (TARGET Eigen3::Eigen)
  add_library(KFilter src/KFilter.cpp)
  target_include_directories(KFilter PUBLIC include)
  
  add_executable(test_filter test_filter.cpp)
  target_link_libraries(test_filter KFilter)
endif (TARGET Eigen3::Eigen)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


