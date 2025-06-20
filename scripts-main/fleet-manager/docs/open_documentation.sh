#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# If fleetman documentation image does not exist, build it
if [[ "$(docker images -q fleetman/docs 2> /dev/null)" == "" ]]; then
	docker build -t fleetman/docs $SCRIPT_DIR
fi

# Copy fleetman API description file
if [ ! -f $SCRIPT_DIR/docs/software/.api.yml ] || ! cmp -s $SCRIPT_DIR/../api/fleet_manager_api.yml $SCRIPT_DIR/docs/software/.api.yml; then
    cp $SCRIPT_DIR/../api/fleet_manager_api.yml $SCRIPT_DIR/docs/software/.api.yml
fi

# Open browser window
xdg-open http://localhost:7070/
# Launch mkdocs
docker run --rm -p 7070:8000 --name fleetman_mkdocs -v $SCRIPT_DIR:/docs fleetman/docs
