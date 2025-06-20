#!/bin/bash
# This script is used to package the groundstation binaries for distribution among other entities

if [[ "$(docker images -q fleetman/gs 2> /dev/null)" == "" ]]; then
	echo "Error: fleetman/gs docker image is not built."
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
mkdir -p $SCRIPT_DIR/fleetman_groundstation/build
mkdir -p $SCRIPT_DIR/fleetman_groundstation/configs
mkdir -p $SCRIPT_DIR/fleetman_groundstation/deploy
mkdir -p $SCRIPT_DIR/fleetman_groundstation/examples

# Copy scripts, configs, dockerfiles and examples
cp $SCRIPT_DIR/../../scripts/run_fleetman_server.sh $SCRIPT_DIR/fleetman_groundstation/build
cp -r $SCRIPT_DIR/../../configs/sensors $SCRIPT_DIR/fleetman_groundstation/configs
cp -r $SCRIPT_DIR/../../configs/plugins $SCRIPT_DIR/fleetman_groundstation/configs
cp $SCRIPT_DIR/../../configs/mosquitto.conf $SCRIPT_DIR/fleetman_groundstation/configs
cp $SCRIPT_DIR/../../configs/server.properties $SCRIPT_DIR/fleetman_groundstation/configs
cp $SCRIPT_DIR/../ros.dockerfile $SCRIPT_DIR/fleetman_groundstation/build
cp $SCRIPT_DIR/../rcljava.dockerfile $SCRIPT_DIR/fleetman_groundstation/build
cp $SCRIPT_DIR/../../deploy/docker-compose.yml $SCRIPT_DIR/fleetman_groundstation/deploy
cp $SCRIPT_DIR/../../deploy/docker-compose.prod.yml $SCRIPT_DIR/fleetman_groundstation/deploy
cp $SCRIPT_DIR/../../deploy/docker-compose.override.yml $SCRIPT_DIR/fleetman_groundstation/deploy
cp $SCRIPT_DIR/../../deploy/launch_groundstation_containers.sh $SCRIPT_DIR/fleetman_groundstation/launch_groundstation_containers.sh
cp $SCRIPT_DIR/../../examples/friends/* $SCRIPT_DIR/fleetman_groundstation/examples
cp $SCRIPT_DIR/../../examples/follow.groovy $SCRIPT_DIR/fleetman_groundstation/examples
cp $SCRIPT_DIR/../../examples/mapping.groovy $SCRIPT_DIR/fleetman_groundstation/examples
cp $SCRIPT_DIR/../../examples/*_with_plugin.groovy $SCRIPT_DIR/fleetman_groundstation/examples

# Edit ground dockerfile to exclude copying source files and building the project
grep -v 'src\|colcon' $SCRIPT_DIR/../ground.dockerfile > $SCRIPT_DIR/fleetman_groundstation/build/ground.dockerfile
sed -i 's/scripts/build/g' $SCRIPT_DIR/fleetman_groundstation/build/ground.dockerfile
# Add COPY instruction to dockerfile to add the binaries
echo "COPY build/install /\$WS_DIR/install" >> $SCRIPT_DIR/fleetman_groundstation/build/ground.dockerfile
echo "COPY build/jars /opt/jardeps/" >> $SCRIPT_DIR/fleetman_groundstation/build/ground.dockerfile

# Change launch script to use the :friends tag for the docker image
sed -i 's/fleetman\/gs/fleetman\/gs:friends/g' $SCRIPT_DIR/fleetman_groundstation/launch_groundstation_containers.sh
sed -i 's/fleetman\/gs/fleetman\/gs:friends/g' $SCRIPT_DIR/fleetman_groundstation/deploy/docker-compose.yml
sed -i 's/$SCRIPT_DIR/$SCRIPT_DIR\/deploy/g' $SCRIPT_DIR/fleetman_groundstation/launch_groundstation_containers.sh

# Run groundstation docker container to copy the groundstation binaries
docker run --rm -dit --name tmp_gs fleetman/gs bash
docker cp tmp_gs:/ws/install/ $SCRIPT_DIR/fleetman_groundstation/build
docker cp tmp_gs:/opt/jardeps/ $SCRIPT_DIR/fleetman_groundstation/build/jars
docker stop tmp_gs

# Create script for building the docker images
printf "SCRIPT_DIR=\"\$( cd \"\$( dirname \"\${BASH_SOURCE[0]}\" )\" &> /dev/null && pwd )\"
docker build -t fleetman/ros:galactic -f \$SCRIPT_DIR/build/ros.dockerfile \$SCRIPT_DIR/.
docker build -t fleetman/rcljava:galactic -f \$SCRIPT_DIR/build/rcljava.dockerfile \$SCRIPT_DIR/.
docker build -t fleetman/gs:friends -f \$SCRIPT_DIR/build/ground.dockerfile \$SCRIPT_DIR/." > $SCRIPT_DIR/fleetman_groundstation/build_docker_image.sh

# Make script executable
chmod +x $SCRIPT_DIR/fleetman_groundstation/build_docker_image.sh

# Compress directory
tar -C $SCRIPT_DIR -czvf $OUTPUT_DIR/fleetman_groundstation.tar.gz fleetman_groundstation

# Remove temporary directory
rm -rf $SCRIPT_DIR/fleetman_groundstation

echo -e "\033[32mPackaged groundstation to file $OUTPUT_DIR/fleetman_groundstation.tar.gz\033[0m"
