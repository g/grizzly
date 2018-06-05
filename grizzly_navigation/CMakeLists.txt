cmake_minimum_required(VERSION 2.8.3)
project(grizzly_navigation)

find_package(catkin)

catkin_package()


if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()


install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
