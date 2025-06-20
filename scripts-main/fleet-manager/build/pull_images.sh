#!/bin/bash
index_array=("ros:galactic" "rcljava:galactic" "drone:latest" "gs:latest" "docs:latest" "ros_bridge:latest" "test:latest" "jmavsim:latest")

for i in "${!index_array[@]}"
do
	# pull image
	docker pull code.nap.av.it.pt:5050/uavs/fleet-manager/"${index_array[i]}"
	# rename image
	docker image tag code.nap.av.it.pt:5050/uavs/fleet-manager/"${index_array[i]}" fleetman/"${index_array[i]}"
	# delete wrongly named image
	docker image rm  code.nap.av.it.pt:5050/uavs/fleet-manager/"${index_array[i]}"
	printf '\n[%d/%d]\n\n' "$((i +1))" "${#index_array[@]}"
done
