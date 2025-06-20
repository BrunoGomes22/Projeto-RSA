#!/bin/bash
# This script is used to package the rosbridge for distribution among other entities

if [[ "$(docker images -q fleetman/ros_bridge 2> /dev/null)" == "" ]]; then
	echo "Error: fleetman/ros_bridge docker image is not built."
	exit
fi

# Directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Set output directory for exporting the .tar.gz
if [ $# -eq 0 ]
  then
  	OUTPUT_DIR=.
else
	if [[ -d $1 ]]; then
		OUTPUT_DIR=$1
	else
		echo "Error: '$1' is not a directory."
		exit
	fi
fi

# Create build and config directories
mkdir -p $SCRIPT_DIR/fleetman_rosbridge/build

# Copy ros packages
cp -r $SCRIPT_DIR/../../pkg/drone_interfaces $SCRIPT_DIR/fleetman_rosbridge/build
cp -r $SCRIPT_DIR/../../pkg/friends_interfaces $SCRIPT_DIR/fleetman_rosbridge/build
cp -r $SCRIPT_DIR/../../vendor/friends/ $SCRIPT_DIR/fleetman_rosbridge/build

# Remove friends binaries
rm -rf $SCRIPT_DIR/fleetman_rosbridge/build/friends/bin

# Copy scripts
cp $SCRIPT_DIR/../../scripts/run_ros1_bridge.sh $SCRIPT_DIR/fleetman_rosbridge/build
cp $SCRIPT_DIR/../../deploy/friends/launch_friends_rosbridge.sh $SCRIPT_DIR/fleetman_rosbridge/launch_rosbridge.sh

# Copy dockerfiles
cp $SCRIPT_DIR/../ros.dockerfile $SCRIPT_DIR/fleetman_rosbridge/build
cp $SCRIPT_DIR/../bridge_ros_ros2.dockerfile $SCRIPT_DIR/fleetman_rosbridge/build

# Add files to dockerfile
sed -i 's/vendor\///g' $SCRIPT_DIR/fleetman_rosbridge/build/bridge_ros_ros2.dockerfile
sed -i 's/pkg\///g' $SCRIPT_DIR/fleetman_rosbridge/build/bridge_ros_ros2.dockerfile
sed -i 's/scripts\///g' $SCRIPT_DIR/fleetman_rosbridge/build/bridge_ros_ros2.dockerfile

# Change launch script to use the :friends tag for the docker image
sed -i 's/fleetman\/ros_bridge/fleetman\/ros_bridge:friends/g' $SCRIPT_DIR/fleetman_rosbridge/launch_rosbridge.sh
sed -i 's/--name friends/--name rosbridge/g' $SCRIPT_DIR/fleetman_rosbridge/launch_rosbridge.sh

# Create script for building the docker images
printf "SCRIPT_DIR=\"\$( cd \"\$( dirname \"\${BASH_SOURCE[0]}\" )\" &> /dev/null && pwd )\"
docker build -t fleetman/ros:galactic -f \$SCRIPT_DIR/build/ros.dockerfile \$SCRIPT_DIR/build
docker build -t fleetman/ros_bridge:friends -f \$SCRIPT_DIR/build/bridge_ros_ros2.dockerfile \$SCRIPT_DIR/build" > $SCRIPT_DIR/fleetman_rosbridge/build_docker_image.sh

# Make script executable
chmod +x $SCRIPT_DIR/fleetman_rosbridge/build_docker_image.sh

# Compress directory
tar -C $SCRIPT_DIR -czvf $OUTPUT_DIR/fleetman_rosbridge.tar.gz fleetman_rosbridge

# Remove temporary directory
rm -rf $SCRIPT_DIR/fleetman_rosbridge

echo -e "\033[32mPackaged rosbridge to file $OUTPUT_DIR/fleetman_rosbridge.tar.gz\033[0m"
