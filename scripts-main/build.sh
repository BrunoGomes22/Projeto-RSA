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
DRONE=$(jq -r '.services.drone.enabled // false' $CONFIG_FILE)

FLEETMANAGER_REPO=$(jq -r '.repositories.fleetmanager.url // "https://code.nap.av.it.pt/uavs/fleet-manager"' $CONFIG_FILE)
FLEETMANAGER_BRANCH=$(jq -r '.repositories.fleetmanager.branch // "master"' $CONFIG_FILE)
DRONEBACKEND_REPOS=$(jq -r '.repositories.dronebackend.url // "https://code.nap.av.it.pt/uavs/drone-backend"' $CONFIG_FILE)
DRONEBACKEND_BRANCH=$(jq -r '.repositories.dronebackend.branch // "master"' $CONFIG_FILE)
DASHBOARD_REPOS=$(jq -r '.repositories.dashboard.url // "https://code.nap.av.it.pt/uavs/dashboard"' $CONFIG_FILE)
DASHBOARD_BRANCH=$(jq -r '.repositories.dashboard.branch // "master"' $CONFIG_FILE)

COLOR='\033[0;36m' # ANSI escape code for COLOR color
NC='\033[0m'       # ANSI escape code to reset color


# ==========================
# ARGUMENTS PARSING
# ==========================


usage() {
  echo "Usage: ./build.sh [-h] [service] [mode]"
  echo "Options:"
    echo -e "  -h:\t\tshow this help message"
    echo -e "  service:\tgroundstation, backend, frontend, simulator, drone"
    echo -e "  mode:\t\t[dev, prod] for backend and frontend"
  exit 1
}

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
	DRONE=false

	# Only build the service passed as argument
	if [ "$1" = "groundstation" ]; then
		GROUNDSTATION=true
	elif [ "$1" = "backend" ]; then
		BACKEND=true
	elif [ "$1" = "frontend" ]; then
		FRONTEND=true
	elif [ "$1" = "simulator" ]; then
		SIMULATOR=true
	elif [ "$1" = "drone" ]; then
		DRONE=true
	else
		usage
	fi
fi


# ==========================
# PROGRAM
# ==========================


sudo -v

# Warning
echo -e "${COLOR}WARNING:${NC}"
echo -e "${COLOR}The building process WILL take several minutes to complete, depending on what services are enabled. Please, be patient.${NC}"

if [[ ! -d "fleet-manager" || ! -d "drone-backend" || ! -d "dashboard" ]]; then 
	echo -e "${COLOR}The repository cloning process that will follow need your attention, since you need to authenticate.${NC}"
	echo ""
fi

# Git credentials cache
git config --global credential.helper cache
git config --global credential.helper 'cache --timeout=180'

# Clone repositories 
if [[ ("$GROUNDSTATION" = "true" || "$SIMULATOR" = "true" || "$DRONE" = "TRUE") && ! -d "fleet-manager" ]]; then
	echo -e "${COLOR}Cloning repository $FLEETMANAGER_REPO${NC}"
	git clone -b $FLEETMANAGER_BRANCH $FLEETMANAGER_REPO
fi
if [[ "$BACKEND" = "true" && ! -d "drone-backend" ]]; then
	echo -e "${COLOR}Cloning repository $DRONEBACKEND_REPOS${NC}"
	git clone -b $DRONEBACKEND_BRANCH $DRONEBACKEND_REPOS
fi
if [[ "$FRONTEND" = "true" && ! -d "dashboard" ]]; then
	echo -e "${COLOR}Cloning repository $DASHBOARD_REPOS${NC}"
	git clone -b $DASHBOARD_BRANCH $DASHBOARD_REPOS
fi

echo ""

# GROUNDSTATION, SIMULATOR & DRONE
if [[ "$GROUNDSTATION" = "true" || "$SIMULATOR" = "true" || "$DRONE" = "true" ]]
then
	cd $SCRIPT_DIR/fleet-manager/build
	if [ "$SIMULATOR" = "false" && "$DRONE" == "false" ]; then
		echo -e "${COLOR}Building groundstation...${NC}"
		bash ./build_docker_images.sh ground
	elif [ "$GROUNDSTATION" = "false" && "$DRONE" == "false" ]; then
		echo -e "${COLOR}Building simulator...${NC}"
		bash ./build_docker_images.sh simulator
	elif [ "$GROUNDSTATION" = "false" && "$SIMULATOR" = "false" ]; then
		echo -e "${COLOR}Building drone...${NC}"
		bash ./build_docker_images.sh drone
	else
		if [ "$DRONE" = "true" && "$SIMULATOR" = "true" && "$GROUNDSTATION" = "true" ]; then
			echo -e "${COLOR}Building groundstation, simulator and drone...${NC}"
			bash ./build_docker_images.sh
		else
			if [ "$DRONE" = "true" ]; then
				echo -e "${COLOR}Building drone...${NC}"
				bash ./build_docker_images.sh drone
			fi

			if [ "$SIMULATOR" = "true" ]; then
				echo -e "${COLOR}Building simulator...${NC}"
				bash ./build_docker_images.sh simulator
			fi

			if [ "$GROUNDSTATION" = "true" ]; then
				echo -e "${COLOR}Building groundstation...${NC}"
				bash ./build_docker_images.sh ground
			fi
		fi
	fi

	if [ "$SIMULATOR" = "true" ]
	then
		cd $SCRIPT_DIR/fleet-manager/tools/sim_launcher
		sudo snap install go --classic
		go build
	fi
fi

cd $SCRIPT_DIR

# BACKEND
if [ "$BACKEND" = "true" ]
then
    BACKEND_MODE=$(jq -r '.services.backend.mode // "prod"' $CONFIG_FILE)
	# if there is $2, use it as mode
	if [ ! -z "$2" ]; then
		BACKEND_MODE=$2
	fi

    echo -e "${COLOR}## Building backend ($BACKEND_MODE)...${NC}"
    if [ "$BACKEND_MODE" == "dev" ]
    then
		VENV=$(jq -r '.services.backend.venv // "venv3615"' $CONFIG_FILE)

	 	if ! command -v pyenv &>/dev/null; then
			echo -e "${COLOR}Development mode requires an old version of Python, version 3.6.15.${NC}"
			echo -e "${COLOR}This is achieved by installing pyenv, which will provide the needed python version.${NC}"
			echo -e "${COLOR}Installing pyenv...${NC}"
			curl https://pyenv.run | bash
		fi

		# Source pyenv initialization script
		export PYENV_ROOT="$HOME/.pyenv"
		export PATH="$PYENV_ROOT/bin:$PATH"
		eval "$(pyenv init -)"
		eval "$(pyenv virtualenv-init -)"

		if ! pyenv versions | grep -q 3.6.15; then
			echo -e "${COLOR}Installing python 3.6.15...${NC}"
			pyenv install 3.6.15
		fi

		if ! pyenv virtualenvs | grep -q $VENV; then
			echo -e "${COLOR}Creating python virtual environment $VENV...${NC}"
			pyenv virtualenv 3.6.15 $VENV
		fi
        
		echo -e "${COLOR}Activating python virtual environment $VENV...${NC}"
		pyenv activate $VENV

		cd $SCRIPT_DIR/drone-backend/Django/GpsTracker
		pip install --upgrade pip
		pip install -r requirements.txt
	elif [ "$BACKEND_MODE" == "prod" ]
	then
		cd $SCRIPT_DIR/drone-backend/Django/GpsTracker
		docker-compose build
	fi
fi

cd $SCRIPT_DIR

# FRONTEND
if [ "$FRONTEND" = true ]
then
    FRONTEND_MODE=$(jq -r '.services.frontend.mode // "prod"' $CONFIG_FILE)
	if [ ! -z "$2" ]; then
		FRONTEND_MODE=$2
	fi

    echo -e "${COLOR}## Building frontend ($FRONTEND_MODE)...${NC}"
	cd $SCRIPT_DIR/dashboard/webportal
    if [ "$FRONTEND_MODE" == "dev" ]
    then
		npm i
    elif [ "$FRONTEND_MODE" == "prod" ]
    then
		docker-compose build
    fi
fi

cd $SCRIPT_DIR

# clear git credentials cache
git credential-cache exit