#!/bin/bash
if [ $START_ROSCORE = true ]; then
	. /opt/ros/noetic/setup.bash && roscore &
	echo -e "\033[32mStarted roscore.\033[0m"
fi
echo -e "\033[32mLaunched ros2/1 bridge.\033[0m"
. /opt/ros/galactic/setup.bash && . /root/bridge_ws/install/setup.bash && ros2 run ros1_bridge dynamic_bridge