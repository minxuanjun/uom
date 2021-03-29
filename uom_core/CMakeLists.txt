cmake_minimum_required(VERSION 3.0.2)
project(uom_core)

set(GFLAGS_PREFER_EXPORTED_GFLAGS_CMAKE_CONFIGURATION TRUE)
set(GLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION TRUE)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

# Build option
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(catkin REQUIRED COMPONENTS
  tf
  cv_bridge
  pcl_ros
  sensor_msgs
  nav_msgs
  visualization_msgs
  message_filters
  uom_msgs)

catkin_package()

find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog 0.3.5 REQUIRED)


add_subdirectory(thirdparty)
include_directories(thirdparty/ros/include)


add_library(uom SHARED
  src/uom/imu/pre_integration.cpp
  src/uom/sim/simple_simulator.cpp
  src/uom/vision/camera_params.cpp
  src/uom/laser/laser_params.cpp
  src/uom/laser/laser_preprocessor.cpp)


target_include_directories(uom PUBLIC
  src
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDES}
  ${OpenCV_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)


target_link_libraries(uom
  ${catkin_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES})


add_executable(simple_simulator_node
  src/uom/sim/simple_simulator_node.cpp)
target_link_libraries(simple_simulator_node uom)


add_executable(laser_preprocessor_node
  src/uom/laser/laser_preprocessor_node.cpp)
target_link_libraries(laser_preprocessor_node uom)


option('BUILD_TESTS' "Build tests" OFF)
if (BUILD_TESTS)
  message(STATUS "Build tests Enabled")
  add_subdirectory(tests)
else ()
  message(STATUS "Build tests Disabled")
endif ()