cmake_minimum_required(VERSION 2.8.3)
project(grizzly_motion)

find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs grizzly_msgs roboteq_msgs roscpp tf diagnostic_updater angles)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS eigen
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_subdirectory(src)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
