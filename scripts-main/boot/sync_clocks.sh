#/bin/bash

GROUNDSTATION_IP=$1

if [ -z "$GROUNDSTATION_IP" ]; then
	echo "Usage: $0 <groundstation_ip>"
	exit 1
fi

echo "Syncing clock with $GROUNDSTATION_IP..."

# sync clocks
while true; do
	ntpdate -u $GROUNDSTATION_IP
	if [ $? -eq 0 ]; then
		break
	fi

	echo "Failed to sync clocks. Trying again..."
	sleep 2
done

echo "Clock synced with $GROUNDSTATION_IP."
timedatectl

exit 0
