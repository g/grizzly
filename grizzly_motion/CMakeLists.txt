cmake_minimum_required(VERSION 2.8.3)
project(grizzly_motion)

find_package(catkin REQUIRED COMPONENTS geometry_msgs grizzly_msgs roscpp tf diagnostic_updater)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS eigen
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_subdirectory(src)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
