#/bin/bash

update_port_conf() {
	sudo sed -i.bak "s|^port: serial:///dev/ttyACM[0-9]*|port: serial:///dev/$1|" $CONFIG_FILE
	echo "Updated $CONFIG_FILE with port: serial://$1"
}

check_container() {
	CONTAINER_ID=$1

	echo "Waiting for drone to launch..."
	# wait for container to launch
	while [ -z "$(docker ps -q -f name=$CONTAINER_ID)" ]; do
		sleep 1
	done
	
	while true; do
		CONTAINER_LOGS=$(docker logs $CONTAINER_ID)

		# loop while not [ERROR] or 'ready on port'
		if echo "$CONTAINER_LOGS" | grep "\[ERROR\]" || echo "$CONTAINER_LOGS" | grep "ready on port"; then
			break
		fi

		sleep 1
	done

	# check for '[ERROR]'
	if echo "$CONTAINER_LOGS" | grep "\[ERROR\]"; then
		echo -e "\e[31mDrone failed to launch.\e[0m"
		
		echo -e "\e[31mRemoving container $CONTAINER_ID\e[0m"
		docker rm -f $CONTAINER_ID

		return 1
	fi

	# check for 'ready on port'
	if echo "$CONTAINER_LOGS" | grep "ready on port"; then
		echo -e "\e[32mDrone launched successfully.\e[0m"
		return 0
	fi
}

get_devices() {
	echo "Searching for USB ACM devices..."
	# get the list of ACM devices
	ACM_DEVICES=$(
		dmesg |								# from dmesg
		grep 'cdc_acm' |					# get the lines with 'cdc_acm ...'
		awk '{print $5}' |	tr -d ':' |		# extract the device name
		grep '^ttyACM' |					# make sure they are ACM devices
		sort -u								# remove duplicates
	)
}

launch_drone() {
	echo "Launching drone container..."
	OUTPUT=$(bash launch_drone_container.sh $DRONE -d -c $CONFIG_FILE 2>&1)
	if echo "$OUTPUT" | grep -q "does not exist"; then
		echo -e "\e[33mDevice from config file does not exist\e[0m"
		return 1
	fi

	CONTAINER_ID=$DRONE
	check_container $CONTAINER_ID

	if [ $? -eq 0 ]; then
		return 0
	fi

	return 1
}

if [ "$DEPLOY_PATH" = "" ]; then
	echo "DEPLOY_PATH is not set. Exiting..."
	exit 1
fi

cd $DEPLOY_PATH

DRONE=${1:-drone01}
CONFIG_FILE="../configs/drone_cfg_serial.yml"

# ==========================
# PORT PROVIDED AS ARGUMENT
# ==========================

if [ ! -z "$2" ]; then
	update_port_conf $2
	launch_drone
	exit 1
fi

# ==========================
# SEARCH FOR ACM DEVICES
# ==========================

while [ -z "$ACM_DEVICES" ]; do
	get_devices

	# if not ACM_DEVICES, then exit
	if [ -z "$ACM_DEVICES" ]; then
		echo -e "\e[32mNo USB ACM devices found.\e[0m"
		echo "Trying again in 5 seconds..."
		sleep 5
		
		continue
	fi

	LAUNCHED=false

	# while LAUNCHED is false
	while [ "$LAUNCHED" = false ]; do
		# loop through the ACM devices
		for ACM in $ACM_DEVICES; do
			echo " "
			echo "Found USB ACM device: $ACM"
			update_port_conf $ACM

			launch_drone

			if [ $? -eq 0 ]; then
				LAUNCHED=true
				break
			fi

			echo "Trying next ACM device..."
		done

		if [ "$LAUNCHED" = false ]; then
			echo " "
			echo -e "\e[33mNo more ACM devices left to try.\e[0m"
			echo "Trying again in 5 seconds..."
			echo " "
			sleep 5

			# update devices
			get_devices
		fi
	done
done
