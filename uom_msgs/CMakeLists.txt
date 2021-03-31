cmake_minimum_required(VERSION 3.0.2)
project(uom_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  sensor_msgs
)

add_message_files(
FILES
  CloudFeature.msg
)


# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
)


catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)