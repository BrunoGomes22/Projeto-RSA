#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [ ! "$(docker ps -q -f name=friends)" ]; then
    echo "ERROR: rosbridge friends container is not running."
    exit
fi

if [ $# -ne 2 ]
	then
		echo "USAGE: ./launch_friends_service.sh [service] [drone_id]"
		echo "Available services: pathfollower, inspection, monitoring"
	exit
fi

case "$1" in
	pathfollower|inspection|monitoring)
		case "$(uname -m)" in
			x86_64)
				docker exec -it friends /bin/bash -c "source /opt/ros/noetic/setup.bash && /root/ws/x64/$1 _drone_id:=$2"
			;;
			aarch64)
				docker exec -it friends /bin/bash -c "source /opt/ros/noetic/setup.bash && /root/ws/arm64/$1 _drone_id:=$2"
			;;
			*)
				echo "ERROR: unknown system architecture: '$(uname -m)'"
			;;
		esac
	;;
	*)
		echo "ERROR: Service must be one of: pathfollower, inspection or monitoring"
	;;
esac
