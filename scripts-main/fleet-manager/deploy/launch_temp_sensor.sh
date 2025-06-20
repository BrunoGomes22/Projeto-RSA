#!/bin/bash

docker run \
    --rm \
    --network host \
    --name temp_sensor \
    -it fleetman/temp_sensor