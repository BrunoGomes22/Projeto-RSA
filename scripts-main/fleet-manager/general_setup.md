# Set Up and Execution

1. Connect to an APU via SSH.

```sh
ssh nap@10.0.22.165 # this ip belongs to the APU with id 136.
```

You will be prompted to enter a password, it should be `openlab`.

*You must be connected to IT's network, be it by VPN or connected physically through an ethernet cable.*

2. If you can't find the `fleet-manager` folder on the APU, fetch the source code from the respective [repository](https://code.nap.av.it.pt/uavs/fleet-manager).

3. Install images for both drone and ground station:

```sh
cd build
./build_docker_images.sh
```

:warning: If you encounter any issues building images, you should pull them from [here](https://code.nap.av.it.pt/uavs/fleet-manager/container_registry) instead. Currently, it's the only way to get them working because of dependency errors.

The following script can be used to facilitate the process. It might already be in the `build` directory as `pull_images.sh`. Be sure to use docker login as is stated in the container registry and do not use sudo. If you do use `sudo docker login`, you also have to run the script with sudo, in order to avoid permission errors in the docker config.json file.

```sh
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
```

4. Finally, to deploy the ground station:

```sh
cd deploy
./launch_groundstation_containers.sh
```

**Use -d for development mode**

## Setup and Launch Simulator (and launch virtual drone)

1. Install Go

```sh
sudo snap install go --classic
```

2. Build the simulator and run simulator

```sh
cd tools/sim_launcher
go build
```

3. Launch virtual drone configuration

```sh
./sim_launcher -c configs/some_drone_config.yml
```

## Launch real drone

```sh
cd deploy
./launch_drone_container.sh drone01 -c ../configs/drone_cfg_serial.yml
```

## Testing with Postman

1. Install Postman

```sh
sudo snap install postman
```

Within Postman do the following

- Go to: File/Import *(top left corner of the Postman program window)*.
- Select:
  - Collection: ```api/fleetman.postman_collection.json``` - collection of drone commands.
  - Environment: ```api/fleetman.postman_environment.json``` - setup variables to be used by Collection.
- Go to: Collections *(sidebar within Postman application)*.
- Click: `No Environment` *(top right corner within Postman application)*.
- Select: `fleetman`.
- Choose what requests to make from within the Collection tab *(Commands, Drone Data, etc...)*.

All set.
