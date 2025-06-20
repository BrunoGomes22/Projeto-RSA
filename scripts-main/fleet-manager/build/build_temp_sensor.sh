#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

docker build -t fleetman/temp_sensor -f $SCRIPT_DIR/temp_sensor.dockerfile $SCRIPT_DIR/..