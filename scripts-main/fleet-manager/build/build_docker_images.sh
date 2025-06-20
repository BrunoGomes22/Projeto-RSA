#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

OPTION=""
if [[ "$@" == *"clean"* ]]; then
  OPTION="--no-cache"
  if [[ "$1" == *"clean"* ]]; then
    set -- ""
  fi
fi


build_drone() {
  echo -e "\033[31mBuilding drone image...\033[0m"
  docker build $OPTION -t fleetman/drone -f $SCRIPT_DIR/drone_x86.dockerfile $SCRIPT_DIR/..
}

build_bridge() {
  echo -e "\033[31mBuilding ROS1/2 bridge image...\033[0m"
  docker build $OPTION -t fleetman/ros_bridge -f $SCRIPT_DIR/bridge_ros_ros2.dockerfile $SCRIPT_DIR/..
}

build_gs () {
  echo -e "\033[31mBuilding rcljava base image...\033[0m"
  docker build $OPTION -t fleetman/rcljava:galactic -f $SCRIPT_DIR/rcljava.dockerfile .
  echo -e "\033[31mBuilding ground station image...\033[0m"
  docker build $OPTION -t fleetman/gs -f $SCRIPT_DIR/ground.dockerfile $SCRIPT_DIR/..
}

build_test() {
  echo -e "\033[31mBuilding ROS test image...\033[0m"
  docker build $OPTION -t fleetman/test -f $SCRIPT_DIR/tests.dockerfile $SCRIPT_DIR/..
}

build_jmavsim() {
  echo -e "\033[31mBuilding jMAVSim image...\033[0m"
  docker build $OPTION -t fleetman/jmavsim -f $SCRIPT_DIR/jmavsim.dockerfile $SCRIPT_DIR/..
}

# If no arguments are provided, the script will build the ground station, drone, and simulator images
if [ $# -eq 0 ]
then
  printf "\033[31mWARN: ROS1 bridge image and test image will only be built by explicitly requesting it, either by "
  printf "running this command with 'bridge', 'test', or 'all' arguments.\033[0m\n"
fi

# The base ROS image is always built
echo -e "\033[31mBuilding ROS base image...\033[0m"
docker build $OPTION -t fleetman/ros:galactic -f $SCRIPT_DIR/ros.dockerfile .

case "$1" in
  drone)
    build_drone
    ;;
  bridge)
    build_bridge
    ;;
  ground)
    build_gs
    ;;
  test)
    build_test
    ;;
  nosim)
    build_drone
    build_gs
    ;;
  simulator)
    build_drone
    build_jmavsim
    ;;
  "")
    build_drone
    build_gs
    build_jmavsim
    ;;
  all)
    build_drone
    build_bridge
    build_gs
    build_test
    build_jmavsim
    ;;
  *)
    echo Error: "$1" filter is not available. Valid filters: drone, ground, test, simulator, bridge, all.
    ;;
esac