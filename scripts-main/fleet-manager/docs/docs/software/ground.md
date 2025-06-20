# Ground station
The ground station package provides support for sending commands to the drones and
receiving telemetry data.

!!swagger .api.yml!!


## Building and installation
<!-- Initial setup -->
A working ROS2 Eloquent environment is required to build and run this module, with the 
addition of the Rcljava in order to use Java. The Spring boot framework was also used to
build the server.

It's recommended to run and build the ground station code with the docker
image provided in the `/build` directory, as Rcljava is unstable and might fail to build
locally. The dockerfile `/build/ground.dockerfile` should be used in that case and the
`build_docker_images.sh`, present in the same directory, can also be used to build both
drone and ground station images.

Rcljava requires gradle to build the dependencies. In order to be able to compile the code
locally without having Rcljava installed while developing in a IDE, there's also a Maven
configuration file. One of the dependencies has to be manually added to Maven, which can be
done by running the script present in `/vendor/install_jar_to_maven_repo.sh`.

In order to build the code, the following command has to be run in the ROS2 workspace
directory:

```bash
$ colcon build --packages-select server
```

To download the dependencies, and extra task was added to the `build.gradle` file and they
can be fetched by running:

```bash
$ gradle -b src/server/build.gradle fetchDeps
```

This has to be executed everytime new dependencies are added to the `build.gradle` file.
<!-- Jars were taken from a build --> 

## Usage
After building the environment, the ground station module can be executed by running:

```bash
$ java -cp /root/ros2_java_ws/install/server/share/server/java/server-1.0-SNAPSHOT.jar:/opt/jardeps/* \
org.nap.fleetman.server.FleetmanServerApplication
```

The `run_fleetman_server.sh` script, available in the `/scripts` directory, can be used
instead of manually typing the command.

The container can be started with the following command:

```bash
$ docker run --network host --rm -it ros2:ground
```

To be able to store the logs locally and run the server with custom properties, run
the `launch_ground_container.sh` script.

```bash
$ ./scripts/launch_ground_container.sh -c configs/server.properties -l logs
```

<!-- Write about application.properties, and logging there -->

Both arguments are optional; the `-c, --config` flag can be used to pass the properties file
and `-l, --logs` to the target log directory. An example properties file can be found in
`/configs/server.properties`. Any Spring Boot application property can be added.


In order to use the docker image as a development workspace, it can be started with the
following command instead:

```bash
$ docker run --network host \
    -v /full/path/to/src/fleet-manager/cmd/drone/:/ws/src/drone
    --rm -it ros2:ground bash
```

The ground API description is documented using OpenAPI and is available in
`/api/fleet_manager_api.yml`.

## Package structure
<!-- Generated with Swagger codegen -->
The ground station server is built using Spring boot. The codebase is divided in
several packages:
* `api`: generated Swagger codegen classes and API controllers
* `core`: drone management and ROS2 message exchange
* `model`: entity models
* `repo`: CRUD repositories
