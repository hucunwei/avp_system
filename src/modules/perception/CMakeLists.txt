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
find_package(PythonLibs 3.8 REQUIRED)

#find_package(Python3 REQUIRED COMPONENTS Development NumPy)
## Set Python source directory
#set(PYTHON_SOURCE_DIR "${CMAKE_SOURCE_DIR}/python")
find_package(PythonLibs 3.6 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        image_transport
        cv_bridge
)

# Locate NumPy headers
execute_process(
        COMMAND python3 -c "import numpy; print(numpy.get_include())"
        OUTPUT_VARIABLE NUMPY_INCLUDE_DIRS
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

#include_directories(
#        include
#        ${catkin_INCLUDE_DIRS}
#        ${Python_INCLUDE_DIRS}
#        ${NUMPY_INCLUDE_DIRS}
#        ${OpenCV_INCLUDE_DIRS}
#)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Python_INCLUDE_DIRS}
    ${NumPy_INCLUDE_DIRS}
)

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
add_definitions(-DIPM_TRAIN_DIR="${PROJECT_SOURCE_DIR}/ipm_train/")

add_library(${PROJECT_NAME}_core
  src/perception.cpp
  src/ipm.cc
)

add_executable(${PROJECT_NAME}_node
  src/perception_node.cpp
  src/python_segmentation.cpp
)

catkin_package(
  # LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS common_msgs
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Python_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node scripts/ config/ models/
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})