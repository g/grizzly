cmake_minimum_required(VERSION 2.8.3)
project(husky_base)

find_package(catkin)

catkin_package()


install(PROGRAMS
  nodes/teleop.py
  nodes/teleop_incremental.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



