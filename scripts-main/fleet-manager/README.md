# Fleet Manager
**Fleet Manager** is a  collection  of modules  developed to  support the  declaration  and
execution of diverse missions for networked fleets of aerial drones.

Communication between  the several network nodes (drones and ground station) is provided by
[**ROS2**](https://index.ros.org/doc/ros2/).  The  modules  are  developed  in  *Java*  and
*C++*, with [**Spring Boot**](https://spring.io/projects/spring-boot)  framework leveraging
the ground server and controlling the drone via [**Mavsdk**](https://mavsdk.mavlink.io/v0.24.0/en/cpp/).
These units can be grouped in two main components: 
 
* The **ground** modules,  currently comprised of the server module, which can receive HTTP
requests with a command and send the appropriate message  to the required drone, while also 
providing an endpoint for accessing the telemetry.  In the future, mission modules  capable
of supporting a mission plan will be available.
* The **drone** modules,  which through  Mavsdk execute the  commands that are received  in
ROS2 messages and publish telemetry data.

For more detailed information, refer to the documentation present in the `/docs` directory.

## v1.10.0

Latest Release

## Drone
- Added **Restrictions** module that includes several movement checks before each command
  - Prevent fly away to Null Island
  - Setup of a Geofence
  - Setup of an altitude limit
- Distinguishing between relative and absolute altitudes with 'height' and 'alt' respectively
- Added `geofence` action command to go to the center of the geofence

## Server
- Added **Pause** and **Resume** Mission endpoints
- The Resume Mission endpoint can take an `action` parameter (passed in the body of the request) to resume the mission in one of the following ways:
  - `last_position` - Resumes the mission from the last position the drone was in before the pause
  - `complete_current` - Completes the current command that was aborted on pause and continues the mission
  - `start_next` - Skips the command that was aborted on pause and continues the mission from the next command

## Project structure
Although this project has a codebase spanning multiple programming languages, the [Standard
Go Project Layout](https://github.com/golang-standards/project-layout)  is  applied in  the
project structure.

* `/api`: OpenAPI specification of the server interface
* `/build`: dockerfiles for building ROS2 environments with the drone and ground modules, 
  as well as simulation and test environments
* `/cmd`: main applications of this project, such as the drone and ground station code
* `/configs`: base configuration files for launching the drone and ground station modules
  and sample plugin and sensor configurations
* `/deploy`: docker-compose files for launching the software modules
* `/docs`: project documentation
* `/examples`: example mission scripts
* `/init`: scripts and systemd services to run in the drone's companion computer
* `/pkg`: code that may be used by external applications, such as ROS2 interfaces or
  assorted software that is not part of the main application
* `/scripts`: scripts for running the different modules and launching the containers
* `/test`: mission scripts to be used when conducting field tests and other test applications
* `/tools`: supporting tools for the project, which aid in simulation and testing
* `/vendor`: built external dependencies, in this case rcljava .jar dependencies which are
  required for compiling ROS2 java code when there isn't a ROS2 installation on the development machine
