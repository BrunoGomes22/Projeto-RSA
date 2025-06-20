#!/bin/bash
if [ $# -eq 0 ]; then
	echo "Usage: $0 DRONE_ID [CONFIG_FILE]"
	echo "	DRONE_ID: id to be attributed to drone"
	echo "	CONFIG_FILE: configuration file location"
	exit 1
fi

cfg_file=$2

add_network_parameters() {
	if command -v ip &> /dev/null
		then
		if [ $# -gt 2 ]; then
			net_interface=$3
		else
			net_interface=$(ls /sys/class/net | grep wl | head -n 1)
		fi
		if [ -n "$net_interface" ]; then
			mac=$(cat /sys/class/net/$net_interface/address)
			ip=$(ip addr show $net_interface | grep -o "inet [0-9]*\.[0-9]*\.[0-9]*\.[0-9]*" | grep -o "[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*")
			if [ -n "$ip" ]; then
				printf "\nipAddress: %s\nmacAddress: %s\n" "$ip" "$mac" >> $file_path
			fi
		fi
	fi
}

if [ $# -eq 1 ]; then
	ros2 run fleetman_drone drone_controller -n $1
else
	file_path=/tmp/$1_cfg.yml
	cp $cfg_file $file_path
	add_network_parameters
	sed -i 's/^/  /' $file_path
	sed -i '1s/^/'$1':\n ros__parameters:\n/' $file_path
	ros2 run fleetman_drone drone_controller -n $1 --ros-args --params-file $file_path
fi

exit_code=$?

while [ $exit_code -eq 5 ]; do
	if [ $# -eq 1 ]; then
		ros2 run fleetman_drone drone_controller -n $1
	else
		ros2 run fleetman_drone drone_controller -n $1 --ros-args --params-file $file_path
	fi
done
