# planning/CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project(planning)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=c++17 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
 common_msgs
  pcl_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# add_message_files(
#   FILES
# #   SensorCalibration.msg
# )

# generate_messages(
#   DEPENDENCIES
#   common_msgs
# )

catkin_package(
  CATKIN_DEPENDS common_msgs
)


catkin_package()

add_executable(${PROJECT_NAME}_node
        src/planning_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})