cmake_minimum_required(VERSION 2.8.3)
project(grizzly_teleop)

find_package(catkin)

catkin_package()

install(PROGRAMS
  nodes/teleop
  nodes/teleop_incremental
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



