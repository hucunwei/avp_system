# common_msgs/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.3)
project(common_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Pose6D.msg
  CanBusData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)