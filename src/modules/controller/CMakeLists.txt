# control/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.3)
project(controller)
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=c++17 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
add_definitions(-DMESH_PATH="${PROJECT_SOURCE_DIR}/meshes/car.dae")
add_definitions(-DDATASET_PATH="${PROJECT_SOURCE_DIR}/data/")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf lgsvl_msgs)

include(cmake/FindEigen.cmake)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)


catkin_package(
  CATKIN_DEPENDS common_msgs lgsvl_msgs
)


add_library(${PROJECT_NAME}_core
  src/controller.cpp
)

add_executable(${PROJECT_NAME}_node
  src/avp_control_node.cpp
  src/ros_viewer/ros_viewer.cpp
  src/KinematicModel.cpp
  src/controller.cpp
  src/avp_map.cpp
  src/slot.cpp
)



target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core ${catkin_LIBRARIES} ${OpenCV_LIBS})


install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})