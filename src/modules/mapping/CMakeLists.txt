# mapping_localization/CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project(avp_mapping)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
add_definitions(-DDATASET_PATH="${PROJECT_SOURCE_DIR}/ipm_data/")

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=c++17 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

include(cmake/FindEigen.cmake)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_core SHARED
    src/pose_interpolation.cpp
    src/avp_mapping.cpp
    src/map.cpp
    src/slot.cpp

    src/subscriber/ipm_seg_image_subscriber.cpp
    src/subscriber/gps_odom_subscriber.cpp

    src/data_pretreat/data_pretreat.cpp
)
target_link_libraries(${PROJECT_NAME}_core ${OpenCV_LIBS})

if ("$ENV{ROS_VERSION}" STREQUAL "1")

    # add_message_files(
    #   FILES
    # #   SensorCalibration.msg
    # )

    # generate_messages(
    #   DEPENDENCIES
    #   common_msgs
    # )

    message(STATUS "Build ros node")
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs tf)
    include_directories(${catkin_INCLUDE_DIRS})

    catkin_package(
      CATKIN_DEPENDS common_msgs
    )

    add_executable(${PROJECT_NAME}_node 
      src/avp_mapping_node_main.cpp 
      src/ros_viewer/mapping_ros_viewer.cpp
      src/ros_viewer/mapping_ros_viewer.cpp
      src/ros_viewer/viewer_interface.cpp
    )

    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})

    install(TARGETS ${PROJECT_NAME}_node
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif ()
