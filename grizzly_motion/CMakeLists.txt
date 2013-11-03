cmake_minimum_required(VERSION 2.8.3)
project(grizzly_motion)

find_package(catkin REQUIRED COMPONENTS geometry_msgs grizzly_msgs roscpp tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)
