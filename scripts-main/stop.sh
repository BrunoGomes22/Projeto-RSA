#!/bin/bash

# ==========================
# INITIAL SETUP
# ==========================


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Install jq if not installed (json parser)
if ! command -v jq &> /dev/null
then
    echo "jq could not be found. Installing..."
    sudo apt-get install jq
fi

CONFIG_FILE="config.json"

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Could not find config file $CONFIG_FILE. Exiting..."
    exit 1
fi

# Read configs
GROUNDSTATION=$(jq -r '.services.groundstation.enabled // false' $CONFIG_FILE)
BACKEND=$(jq -r '.services.backend.enabled // false' $CONFIG_FILE)
FRONTEND=$(jq -r '.services.frontend.enabled // false' $CONFIG_FILE)
SIMULATOR=$(jq -r '.services.simulator.enabled // false' $CONFIG_FILE)

SIM_DIR=$(jq -r '.services.simulator.dir // "fleet-manager/tools/sim_launcher"' $CONFIG_FILE)
GS_DIR=$(jq -r '.services.groundstation.dir // "fleet-manager"' $CONFIG_FILE)
BACKEND_DIR=$(jq -r '.services.backend.dir // "drone-backend"' $CONFIG_FILE)
GPSTRACKER_DIR="$BACKEND_DIR/Django/GpsTracker"
FRONTEND_DIR=$(jq -r '.services.frontend.dir // "dashboard"' $CONFIG_FILE)
WEBPORTAL_DIR="$FRONTEND_DIR/webportal"
BACKEND_MODE=$(jq -r '.services.backend.mode // "prod"' $CONFIG_FILE)
FRONTEND_MODE=$(jq -r '.services.frontend.mode // "prod"' $CONFIG_FILE)

COLOR='\033[0;31m'   # ANSI escape code for COLOR color
NC='\033[0m'       # ANSI escape code to reset color


# ==========================
# ARGUMENTS PARSING
# ==========================

# TODO: Add support for stopping drone service

usage() {
    echo "Usage: ./stop.sh [-h] [service] [mode]"
    echo "Options:"
    echo -e "  -h:\t\tshow this help message"
    echo -e "  service:\tgroundstation, backend, frontend, simulator"
    echo -e "  mode:\t\t[dev, prod] for backend and frontend"
    exit 1
}

USE_TERMINAL=false

while getopts ":h" arg; do
    case $arg in
        \?)
            usage
            ;;
        h)
            usage
            ;;
    esac
done

# Shift the options out of the argument list
shift $((OPTIND - 1))

# If an argument is passed, clear all flags
if [ $# -gt 0 ]; then
	GROUNDSTATION=false
	BACKEND=false
	FRONTEND=false
	SIMULATOR=false

    # Only build the service passed as argument
    if [ "$1" = "groundstation" ]; then
        GROUNDSTATION=true
    elif [ "$1" = "backend" ]; then
        BACKEND=true
    elif [ "$1" = "frontend" ]; then
        FRONTEND=true
    elif [ "$1" = "simulator" ]; then
        SIMULATOR=true
    else
        usage
    fi
fi


# ==========================
# PROGRAM
# ==========================


# FRONTEND
if [ "$FRONTEND" = "true" ]
then
	if [ ! -z "$2" ]; then
		FRONTEND_MODE=$2
	fi
	
	echo -e "${COLOR}## Stopping fronted ($FRONTEND_MODE)...${NC}"
	if [ "$FRONTEND_MODE" = "dev" ]
	then
		WEBPORTAL_PID=$(ps aux | grep webportal | grep -v grep | awk '{print $2}')
		if [ ! -z "$WEBPORTAL_PID" ]; then
			kill -INT $WEBPORTAL_PID
		fi
	elif [ "$FRONTEND_MODE" = "prod" ]
	then
		cd $SCRIPT_DIR/$WEBPORTAL_DIR; docker-compose down
	fi
fi

# BACKEND
if [ "$BACKEND" == "true" ]
then
 	if [ ! -z "$2" ]; then
		BACKEND_MODE=$2
	fi

	echo -e "${COLOR}## Stopping backend ($BACKEND_MODE)...${NC}"
	if [ "$BACKEND_MODE" = "dev" ]
	then
		# stop celery workers
		echo "Stopping celery workers..."
		pkill -f "python3.6 /home/nap_uavs/.pyenv/versions/venv3615/bin/celery -A GpsTracker worker -B"

		# stop rabbitmq container
		RABBITMQ_CONTAINER_ID=$(docker ps --filter "ancestor=rabbitmq" -q)
		if [ ! -z "$RABBITMQ_CONTAINER_ID" ]; then
			echo "Stopping rabbitmq container..."
			docker stop $RABBITMQ_CONTAINER_ID
			docker rm $RABBITMQ_CONTAINER_ID
		fi

		# stop django server
		DJANGO_PID=$(ps aux | grep manage.py | grep -v grep | awk '{print $2}')
		if [ ! -z "$DJANGO_PID" ]; then
			kill -INT $DJANGO_PID
		fi
	elif [ "$BACKEND_MODE" = "prod" ]
	then
		cd $SCRIPT_DIR/$GPSTRACKER_DIR; docker-compose down
	fi
fi

# SIMULATOR
if [ "$SIMULATOR" = "true" ]
then
	echo -e "${COLOR}## Stopping simulator...${NC}"
	SIMULATOR_PID=$(ps aux | grep sim_launcher | grep -v grep | awk '{print $2}')
	if [ ! -z "$SIMULATOR_PID" ]; then
		kill -INT $SIMULATOR_PID
	fi
fi

# GROUNDSTATION
if [ "$GROUNDSTATION" = "true" ]
then
	echo -e "${COLOR}## Stopping groundstation...${NC}"
	cd $SCRIPT_DIR/$GS_DIR/deploy; docker compose down
fi
