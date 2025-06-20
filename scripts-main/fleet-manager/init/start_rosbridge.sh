#!/bin/bash
source ~/.bashrc
source /opt/ros/melodic/setup.bash
source /opt/ros/eloquent/local_setup.bash
source $ROS_BRIDGE_PATH/ros1_bridge/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics