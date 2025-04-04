# planning/CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project(planning)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=c++17 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  common_msgs
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

add_library(${PROJECT_NAME}_core
  src/planning.cpp
)

add_executable(${PROJECT_NAME}_node
  src/planning_node.cpp
)

catkin_package(
  CATKIN_DEPENDS common_msgs
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})