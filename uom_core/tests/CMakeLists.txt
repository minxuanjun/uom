cmake_minimum_required(VERSION 3.10)

if(NOT TARGET gtest_main)
    add_subdirectory(googletest EXCLUDE_FROM_ALL)
endif(NOT TARGET gtest_main)

add_executable(test_uom
  src/test_uom.cpp
  src/test_imu_params.cpp
  src/test_camera_params.cpp
  src/test_integration.cpp)

target_link_libraries(test_uom uom gtest gtest_main)