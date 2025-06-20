#!/bin/bash
if [[ "$(docker images -q fleetman/gs 2> /dev/null)" == "" ]]; then
	echo "Error: fleetman/drone docker image is not built."
	exit
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "Starting ground container..."
docker run --rm -d --name tmp_ground fleetman/gs
echo "Copying rcljava jars..."
docker cp tmp_ground:/ws/install/drone_interfaces/share/drone_interfaces/java/drone_interfaces_messages.jar $SCRIPT_DIR
docker cp tmp_ground:/ws/install/friends_interfaces/share/friends_interfaces/java/friends_interfaces_messages.jar $SCRIPT_DIR
docker cp tmp_ground:/root/ros2_java_ws/install/builtin_interfaces/share/builtin_interfaces/java/builtin_interfaces_messages.jar $SCRIPT_DIR
docker cp tmp_ground:/root/ros2_java_ws/install/rcl_interfaces/share/rcl_interfaces/java/rcl_interfaces_messages.jar $SCRIPT_DIR
docker cp tmp_ground:/root/ros2_java_ws/install/std_msgs/share/std_msgs/java/std_msgs_messages.jar $SCRIPT_DIR
docker cp tmp_ground:/root/ros2_java_ws/install/rcljava/share/rcljava/java/rcljava.jar $SCRIPT_DIR
docker cp tmp_ground:/root/ros2_java_ws/install/rcljava_common/share/rcljava_common/java/rcljava_common.jar $SCRIPT_DIR
echo "Stopping ground container..."
docker kill tmp_ground