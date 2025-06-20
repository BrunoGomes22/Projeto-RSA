#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo -e "\033[31mBuilding ROS base image...\033[0m"
docker build -t fleetman/ros:galactic -f $SCRIPT_DIR/../ros.dockerfile .
echo -e "\033[31mBuilding drone image...\033[0m"
docker build -t fleetman/drone -f $SCRIPT_DIR/drone_arm64.dockerfile $SCRIPT_DIR/../..