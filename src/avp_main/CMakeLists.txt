cmake_minimum_required(VERSION 2.8.3)
project(avp_main)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs
#   pcl_ros visualization_msgs
#   dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS
  # LIBRARIES
  #   perception_core
 
  CATKIN_DEPENDS
    roscpp std_msgs
)