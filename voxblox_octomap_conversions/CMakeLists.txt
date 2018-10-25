cmake_minimum_required(VERSION 2.8.3)
project(voxblox_octomap_conversions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)


cs_add_executable(voxblox_octomap_converter
        src/voxblox_octomap_converter_node.cpp
        )

##########
# EXPORT #
##########


cs_install()
cs_export()
