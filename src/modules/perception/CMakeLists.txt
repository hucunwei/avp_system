# perception/CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project(perception)


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=c++17 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  std_msgs
  common_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

# add_message_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   common_msgs
# )
add_definitions(-DCONFIG_DIR="${PROJECT_SOURCE_DIR}/config/")
add_definitions(-DIPM_RAW_DIR="${PROJECT_SOURCE_DIR}/ipm_raw/")
add_definitions(-DIPM_LABEL_DIR="${PROJECT_SOURCE_DIR}/ipm_label/")

add_library(${PROJECT_NAME}_core
  src/perception.cpp
  src/ipm.cc
)

add_executable(${PROJECT_NAME}_node
  src/perception_node.cpp
)

catkin_package(
  # LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS common_msgs
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})