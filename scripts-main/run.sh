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

SIM_DIR=$(jq -r '.services.simulator.dir // "fleet-manager/tools/sim_launcher"' $CONFIG_FILE)
DRONE_DIR=$(jq -r '.services.drone.dir // "fleet-manager/deploy"' $CONFIG_FILE)
DRONE_ID=$(jq -r '.services.drone.id // "drone01"' $CONFIG_FILE)
GS_DIR=$(jq -r '.services.groundstation.dir // "fleet-manager"' $CONFIG_FILE)
BACKEND_DIR=$(jq -r '.services.backend.dir // "drone-backend"' $CONFIG_FILE)
GPSTRACKER_DIR="$BACKEND_DIR/Django/GpsTracker"
FRONTEND_DIR=$(jq -r '.services.frontend.dir // "dashboard"' $CONFIG_FILE)
WEBPORTAL_DIR="$FRONTEND_DIR/webportal"
BACKEND_MODE=$(jq -r '.services.backend.mode // "prod"' $CONFIG_FILE)
FRONTEND_MODE=$(jq -r '.services.frontend.mode // "prod"' $CONFIG_FILE)
GROUNDSTATION_MODE=$(jq -r '.services.groundstation.mode // "prod"' $CONFIG_FILE)

COLOR='\033[0;32m' # ANSI escape code for COLOR color
NC='\033[0m'       # ANSI escape code to reset color


# ==========================
# ARGUMENTS PARSING
# ==========================


usage() {
    echo "Usage: ./run.sh [-ut] [-h] [service] [mode]"
    echo "Options:"
    echo -e "  -t:\t\trun the services in separate terminals"
    echo -e "  -u:\t\tupdate the services before running (groundstation)"
    echo -e "  -h:\t\tshow this help message"
    echo -e "  service:\tgroundstation, backend, frontend, simulator, drone"
    echo -e "  mode:\t\t[dev, prod] for backend, frontend and groundstation"
    echo -e "  \t\t<1-3> drones for simulator"
    echo -e "  \t\t<drone_id> for drone"
    exit 1
}

USE_TERMINAL=false
UPDATE=false

while getopts ":hut" arg; do
    case $arg in
    	t)
            USE_TERMINAL=true
            ;;
        u)
            UPDATE=true
            ;;
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


# Function to cleanup and exit the script
cleanup_and_exit() {
    echo "Script execution interrupted. Exiting..."
    exit 1
}

# Trap the interrupt signal (SIGINT) and call the cleanup function
trap cleanup_and_exit INT

# SIMULATOR
if [ "$SIMULATOR" = true ]
then
    # if there is $2 and it is a number from 0 to 3, use it as the number of drones
    if [ ! -z "$2" ] && [ "$2" -gt 0 ] && [ "$2" -le 3 ]; then
        N_DRONES=$2
    else
        N_DRONES=0
        DRONE_FILE=$(jq -r '.services.simulator.file // "single_drone_aveiro.yml"' $CONFIG_FILE)
    fi
    
    case $N_DRONES in
    	1)
            DRONE_FILE="single_drone_aveiro.yml"
            ;;
        2)
            DRONE_FILE="two_drones_aveiro.yml" 
            ;;
        3)
            DRONE_FILE="three_drones_aveiro.yml"
            ;;
    esac

    echo -e "${COLOR}## Starting simulator...${NC}"
    COMMAND="$SIM_DIR/sim_launcher"
    if [ "$DRONE_FILE" != "" ]; then
        echo "Using $DRONE_FILE"
        COMMAND="$COMMAND -c $SIM_DIR/configs/$DRONE_FILE"
    fi
    
    if [ "$USE_TERMINAL" = true ]; then
        gnome-terminal --title="SIMULATOR" -- bash -c "$COMMAND"
    else
        eval "$COMMAND &"
    fi
fi

# DRONE
if [ "$DRONE" = true ]
then
    if [ ! -z "$2" ]; then
        DRONE_ID=$2
    fi

    DRONE_LAUNCHER="$SCRIPT_DIR/boot/launch_drone.sh"
    if [ ! -f "$DRONE_LAUNCHER" ]; then
        echo "Could not find drone launcher script $DRONE_LAUNCHER. Skipping drone launch..."
    else
        echo -e "${COLOR}## Launching drone $DRONE_ID...${NC}"
        COMMAND="DEPLOY_PATH=$SCRIPT_DIR/$DRONE_DIR $DRONE_LAUNCHER $DRONE_ID"
        if [ "$USE_TERMINAL" = true ]; then
            gnome-terminal --title="DRONE" -- bash -c "$COMMAND"
        else
            eval "$COMMAND &"
        fi
    fi
fi

# GROUNDSTATION
if [ "$GROUNDSTATION" = true ]
then
    if [ ! -z "$2" ]; then
        GROUNDSTATION_MODE=$2
    fi

    # Update groundstation
    if [ "$UPDATE" = true ]; then
        echo -e "${COLOR}## Updating groundstation...${NC}"
        COMMAND="cd $SCRIPT_DIR/$GS_DIR; git pull"
        eval "$COMMAND"

        # if prod mode, build the containers
        if [ "$GROUNDSTATION_MODE" == "prod" ]; then
            echo -e "${COLOR}## Re-building groundstation containers...${NC}"
            COMMAND="cd $SCRIPT_DIR/$GS_DIR/build; docker build -t fleetman/gs -f ground.dockerfile .."
            eval "$COMMAND"
        fi
    fi

    echo -e "${COLOR}## Starting groundstation ($GROUNDSTATION_MODE)...${NC}"
    if [ "$GROUNDSTATION_MODE" == "dev" ]
    then
        COMMAND="cd $SCRIPT_DIR/$GS_DIR/deploy; bash launch_groundstation_containers.sh -d"
    elif [ "$GROUNDSTATION_MODE" == "prod" ]; then
        COMMAND="cd $SCRIPT_DIR/$GS_DIR/deploy; bash launch_groundstation_containers.sh"
    fi

    if [ "$USE_TERMINAL" = true ]; then
        gnome-terminal --title="GROUNDSTATION" -- bash -c "$COMMAND"
    else
        eval "$COMMAND &"
    fi
fi

# BACKEND
if [ "$BACKEND" = true ]
then
    port_kill() {
        # Check if the port is in use
        if netstat -tuln | grep ":$1 " &>/dev/null; then
            # Port is in use, so kill the process using the port
            sudo fuser -n tcp -k $1
            echo "Port $1 was busy and the process using it has been killed."
        fi
    }

    port_kill 8000

    if [ ! -z "$2" ]; then
        BACKEND_MODE=$2
    fi

    echo -e "${COLOR}## Starting backend ($BACKEND_MODE)...${NC}"
    if [ "$BACKEND_MODE" == "dev" ]
    then
        echo "  Running in development mode"

        # Source pyenv initialization script
        export PYENV_ROOT="$HOME/.pyenv"
        export PATH="$PYENV_ROOT/bin:$PATH"
        eval "$(pyenv init -)"
        eval "$(pyenv virtualenv-init -)"
        
        # Activate virtualenv if not already activated
        if [ -z "$VIRTUAL_ENV" ]; then
            echo "  Activating python virtual environment..."
            VENV=$(jq -r '.services.backend.venv // "venv3615"' $CONFIG_FILE)
            pyenv activate venv3615
        fi

        # start mongo database if not running
        if ! pgrep -x "mongod" > /dev/null
        then
            echo "  MongoDB is not running. Starting..."
            sudo service mongod start
        fi

        COMMAND="cd $SCRIPT_DIR/$BACKEND_DIR; bash run_drone_backend.sh"
        if [ "$USE_TERMINAL" = true ]; then
            gnome-terminal --title="BACKEND" -- bash -c "$COMMAND"
        else
            echo "  Running backend..."
            eval "$COMMAND &"
        fi

        # start celery workers
        COMMAND="cd $SCRIPT_DIR/$BACKEND_DIR; bash run_celery.sh"
        if [ "$USE_TERMINAL" = true ]; then
            gnome-terminal --title="CELERY" -- bash -c "$COMMAND"
        else
            echo "  Running celery workers..."
            eval "$COMMAND &"
        fi
    elif [ "$BACKEND_MODE" == "prod" ]; then
        echo "  Starting containers"
        COMMAND="cd $SCRIPT_DIR/$GPSTRACKER_DIR; docker-compose up --build"
        if [ "$USE_TERMINAL" = true ]; then
            gnome-terminal --title="BACKEND" -- bash -c "$COMMAND"
        else
            eval "$COMMAND &"
        fi
    fi
fi

# FRONTEND
if [ "$FRONTEND" = true ]
then
    if [ ! -z "$2" ]; then
        FRONTEND_MODE=$2
    fi

    echo -e "${COLOR}## Starting frontend ($FRONTEND_MODE)...${NC}"
    if [ "$FRONTEND_MODE" == "dev" ]
    then
        echo "  Running in development mode"
        COMMAND="cd $SCRIPT_DIR/$FRONTEND_DIR; bash run_frontend.sh"
        if [ "$USE_TERMINAL" = true ]; then
            gnome-terminal --title="FRONTEND" -- bash -c "$COMMAND"
        else
            eval "$COMMAND &"
        fi
    elif [ "$FRONTEND_MODE" == "prod" ]; then
        echo "  Starting containers"
        COMMAND="cd $SCRIPT_DIR/$WEBPORTAL_DIR; docker-compose up --build"
        if [ "$USE_TERMINAL" = true ]; then
            gnome-terminal --title="FRONTEND" -- bash -c "$COMMAND"
        else
            eval "$COMMAND &"
        fi
    fi
fi