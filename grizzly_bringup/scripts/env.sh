#!/usr/bin/env bash

# External roslaunch helper script
# Place this on any non-master computers which need nodes launched by the master on startup
# For more details, consult the roslaunch docs as well as the remote_machine.launch file
# in grizzly_bringup/launch/core

source /home/administrator/ros/setup.bash
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_IP=192.168.1.101

exec "$@"
