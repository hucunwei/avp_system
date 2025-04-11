# mapping_localization/CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project(localization)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=c++17 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf common_msgs)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

include(cmake/FindEigen.cmake)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# add_message_files(
#   FILES
# #   SensorCalibration.msg
# )

# generate_messages(
#   DEPENDENCIES
#   common_msgs
# )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS common_msgs
)


add_library(${PROJECT_NAME}_core 
  src/ekf.cpp
  src/frame.cpp
  src/localization.cpp
  src/map.cpp
  src/slot.cpp
  src/frame.cpp
  # src/publisher/*.cpp
  src/subscriber/chassis_data_subscriber.cpp
  src/subscriber/ipm_seg_image_subscriber.cpp
  src/subscriber/gps_odom_subscriber.cpp

  src/publisher/pose_publisher.cpp

  src/data_pretreat/data_pretreat.cpp

  src/map_viewer/map_viewer.cpp
)

target_link_libraries(${PROJECT_NAME}_core ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node
  
  src/localization_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PROJECT_NAME}_core)

catkin_package(
  CATKIN_DEPENDS common_msgs
)


install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})