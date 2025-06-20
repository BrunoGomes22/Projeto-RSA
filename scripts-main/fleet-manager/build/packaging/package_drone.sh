#!/bin/bash
# This script is used to package the drone software for distribution among other entities

if [[ "$(docker images -q fleetman/drone 2> /dev/null)" == "" ]]; then
	echo "Error: fleetman/drone docker image is not built."
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
mkdir -p $SCRIPT_DIR/fleetman_drone/build
mkdir -p $SCRIPT_DIR/fleetman_drone/configs

# Copy scripts and configs
cp $SCRIPT_DIR/../../scripts/run_drone_controller.sh $SCRIPT_DIR/fleetman_drone/build
cp $SCRIPT_DIR/../../deploy/launch_drone_container.sh $SCRIPT_DIR/fleetman_drone/
cp $SCRIPT_DIR/../../configs/drone_*  $SCRIPT_DIR/fleetman_drone/configs

# If running on arm64 architecture, also copy MAVSDK binaries, which should be available at the vendor/mavsdk directory
if [ "$(uname -m)" = "aarch64" ]; then
	cp -r $SCRIPT_DIR/../../vendor/mavsdk $SCRIPT_DIR/fleetman_drone/build/
	ARCH_LABEL=arm64
else
	ARCH_LABEL=x86
fi

# Copy dockerfiles
cp $SCRIPT_DIR/../ros.dockerfile $SCRIPT_DIR/fleetman_drone/build
cp $SCRIPT_DIR/drone_friends_$ARCH_LABEL.dockerfile $SCRIPT_DIR/fleetman_drone/build/drone.dockerfile

# Run drone docker container to copy the drone_controller binaries
docker run --rm -dit --name tmp_drone fleetman/drone bash
docker cp tmp_drone:/ws/install/ $SCRIPT_DIR/fleetman_drone/build
docker stop tmp_drone

# Change launch script to use the :friends tag for the docker image
sed -i 's/fleetman\/drone/fleetman\/drone:friends/g' $SCRIPT_DIR/fleetman_drone/launch_drone_container.sh

# Create script for building the docker images
printf "SCRIPT_DIR=\"\$( cd \"\$( dirname \"\${BASH_SOURCE[0]}\" )\" &> /dev/null && pwd )\"
docker build -t fleetman/ros:galactic -f \$SCRIPT_DIR/build/ros.dockerfile .
docker build -t fleetman/drone:friends -f \$SCRIPT_DIR/build/drone.dockerfile \$SCRIPT_DIR/." > $SCRIPT_DIR/fleetman_drone/build_docker_image.sh

# Make script executable
chmod +x $SCRIPT_DIR/fleetman_drone/build_docker_image.sh

# Compress directory
tar -C $SCRIPT_DIR -czvf $OUTPUT_DIR/fleetman_drone_$ARCH_LABEL.tar.gz fleetman_drone

# Remove temporary directory
rm -rf $SCRIPT_DIR/fleetman_drone

echo -e "\033[32mPackaged drone software for $ARCH_LABEL architecture to file $OUTPUT_DIR/fleetman_drone_$ARCH_LABEL.tar.gz\033[0m"
