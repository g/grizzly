cmake_minimum_required(VERSION 2.8.3)
project(grizzly_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  Ambience.msg
  Indicators.msg
  Status.msg
)
generate_messages(
  DEPENDENCIES std_msgs
)
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
