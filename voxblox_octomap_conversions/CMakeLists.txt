cmake_minimum_required(VERSION 2.8.3)
project(voxblox_octomap_conversions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(octomap REQUIRED)

include_directories(${OCTOMAP_INCLUDE_DIRS})


add_definitions(-std=c++11 -Wall -Wextra)


cs_add_executable(voxblox_octomap_converter
        src/voxblox_octomap_converter_node.cpp
        )
target_link_libraries(voxblox_octomap_converter ${OCTOMAP_LIBRARIES})

##########
# EXPORT #
##########


cs_install()
cs_export()
