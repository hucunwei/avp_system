cmake_minimum_required(VERSION 2.8.3)
project(avp_system)

# 包含子模块
add_subdirectory(modules/common_msgs)
add_subdirectory(modules/perception)
add_subdirectory(modules/mapping_localization)
add_subdirectory(modules/planning)
add_subdirectory(modules/control)
