#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
docker run -it --rm --network host --name friends -v $SCRIPT_DIR/../../vendor/friends/bin:/root/ws fleetman/ros_bridge