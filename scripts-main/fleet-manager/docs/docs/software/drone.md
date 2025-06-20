# Drone core
The drone core package provides support for receiving a command in a ROS2 message and
executing it using the MAVSDK library, and publishing telemetry updates.

## Building and installation
<!-- Initial setup -->
A working ROS2 Eloquent environment is required to build and run this module, as well as
the **MAVSDK** and **Jsoncpp** libraries. These dependencies can be installed manually, or a
ready-to-run docker image can be built by using the provided dockerfile in
`/build/drone.dockerfile`. A script, `build_docker_images.sh`, present in the same
directory, can also be used to build both drone and ground station images.

When building the package and dependencies manually instead of using the provided docker
image, the following commands should be executed in the ROS2 workspace directory:

```bash
$ colcon build
$ source install/setup.bash
```

The Pixhawk flight controller should also be reachable by the "drone machine" and
properly configured. In case no physical flight controller is available, a compatible
simulator may be used instead, for example, jMavSim or Gazebo.

## Usage
### Run manually
After building the environment, the drone module can be executed by running:

```bash
$ ros2 run fleetman_drone drone_controller -n DRONE_ID --ros-args --params-file cfg.yml
```

While the `-n` (node/drone identifier) flag and value must be present in order to assert
how to identify this drone, the trailing arguments are used to load different parameters.
Omitting these will result in the program running with the default values. This parameter
file can be written following the ROS2 parameter file conventions, and the target node name
in the file should match the one provided with the `-n` flag.

In order to simplify this process, the `/scripts` directory contains the
`run_drone_controller.sh` script, which can be run in the following manner:

```bash
$ ./run_drone_controller.sh $DRONE_ID cfg.yml
```

This example assumes a `$DRONE_ID` environment variable is present, but any string value
can be used. The config file argument may be omitted to use the default parameters instead.

### Run using Docker 
In case the drone docker image is being used, the container will start executing this
module by default. This is the base form for starting the container:

```bash
$ docker run --network host --rm -it ros2:drone
```

The network host mode is required to allow the ports to be exposed to communicate with the
ground station. This will use the default configurations and as such, will connect to the
Pixhawk through UDP. Physical access to the Pixhawk and custom parameters can also be provided:

```bash
$ docker run --network host \
  --privileged \
  -v /dev/ttyPixhawk:/dev/ttyPixhawk \
  -v /full/path/to/drone_cfg.yml:/ws/drone_cfg.yml
  -e CFG_FILE=drone_cfg.yml \
  --rm -it ros2:drone
```

The `privileged` flag and the first volume mapping are necessary to access the Pixhawk's
serial port, in this example assuming it is accessible through `/dev/ttyPixhawk`. The
second volume points to the drone configuration file and then the `$CFG_FILE` environment
variable also has to be set.

#### Launch script
A script, `launch_drone_container.sh`, is also available in the `/scripts` directory,
which can be used instead of manually providing these flags. As an example:

```bash
$ ./scripts/launch_drone_container.sh $DRONE_ID -c configs/drone_cfg_serial.yml -s
```

Only the `$DRONE_ID` value is mandatory. The `-c, --config` flag indicates the path to the
drone configuration file and the `-s, --serial` flag whether the serial port is used, which
is false by default. This mode relies on the port to be defined in the configuration file,
if one is provided. Example configuration files are present in the `/configs` directory.

#### Using Docker as development environment
In order to use the docker image as a development workspace, it can be started with the
following command instead:

```sh
docker run --network host \
    -v /full/path/to/src/fleet-manager/cmd/drone/:/ws/src/drone
    --rm -it ros2:drone bash
```

This will share the current state of the source code from the host machine with the
container, while also having `bash` as an entrypoint instead of running the module. You
should run `colcon build` on the ROS2 workspace directory every time the source code is
modified. Note that this example assumes the Pixhawk will be connected through a UDP port;
in case a serial connection is desired, the device should also be added as a volume.

## Interfaces
### Configurable parameters
<!-- TODO provide example config--> 
The available drone parameters and their default values are:

| Parameter        | Default value | Description                                                                                     |
|------------------|---------------|-------------------------------------------------------------------------------------------------|
| port             | udp://:14540  | The port where the Pixhawk is connected (can be a serial port such as /dev/ttyPixhawk or a url) |
| telemetryTopic   | /telem        | Topic where the telemetry messages will be published                                            |
| commandTopic     | /cmd          | Topic where the system will retrieve the drone commands from                                   |
| statusTopic      | /status       | Topic where system status and action completion will be published                            |
| paramTopic       | /param        | Topic to receive messages with parameters to be altered                                        |
| telemetryRateMs  | 1000          | Rate at which telemetry messages will be published, in milliseconds                             |
| acceptanceRadius | 0.5           | Distance in meters from target point at which it is considered reached                             |
| maxSpeed         | 5.0           | Max speed in meters/second while using go to commands                                              |
| maxAltitude      | 10.0         | Maximum altitude the drone can reach (in meters)                           |
| takeoffAltitude  | 15.0          | Relative takeoff altitude (in meters above ground)                                              |
| returnAltitude   | 15.0          | Relative minimum altitude the drone has to reach when in return to launch mode                 |

All of these can be set in a YAML file to be loaded by the program when starting. If one of
the parameters isn't defined in the file, the default value will be assumed. The "maxSpeed",
"takeoffAltitude" and "returnAltitude" parameters can be changed in runtime by sending a
json formatted ROS2 message to the parameter topic (`/param` by default).

```json
{
  "droneId":"drone01",
  "params": [
    {
      "name":"max_speed",
      "value":10.0
    },
    {
      "name":"return_altitude",
      "value":50.0
    },
    {
      "name":"takeoff_altitude",
      "value":20.0
    }
  ]
}
```
Only the parameters that are to be changed need to be included in the message. Max speed and
takeoff altitude must be greater than 0.

### Implementation and interface
The MAVlink library used to interface with the Pixhawk is the MAVSDK C++ library. Currently,
the Action, Offboard and Telemetry plugins have most of their functionalities exposed through
ROS2. For example, a message can be sent to execute an “action” or “offboard” command, while
the telemetry status is also sent to another topic.

In order to send a command to the drone, a message has to be published in the previously
configured command topic, the default being /cmd. Similarly, to receive the telemetry updates
it’s necessary to subscribe to the telemetry topic, the default being /telem.

### Message format
The exchanged ROS2 messages are of the type std_msgs/String, composed in JSON format. The
JsonCpp library is used to parse/create JSON strings in C++.

An important aspect to take into consideration is that these strings are case
sensitive and must be in lower case, but the order in which the parameters are
presented is irrelevant. As in any valid JSON, strings must be wrapped in quotes.

#### Command message format

```json
{
    "droneId": "drone01",
    "mode": "control_mode",
    "cmd": "command",
    "...": "..."
}
```

Data types:
- droneId: string
- mode: string (either “action” or “offboard”)
- cmd: string
- Other params: float

The droneId must be the one defined at launch time and the mode indicates whether
the command is part of the action or offboard plugins of MAVSDK. The accepted
commands (cmd) depend on the chosen mode and are described on the next section. Some 
commands take additional parameters.

### Command action
There are 6 action command types that are accepted:
- arm - arm the drone
- disarm  - disarm the drone
- takeoff  - takeoff the drone
- land  - land the drone at the current position
- goto  - move the drone to a coordinate
- return - move the drone to the takeoff position and then land


For example, if one is trying to send an arm command, a message with the following
contents has to be sent:

```json
{
    "droneId": "drone01",
    "mode": "action",
    "cmd": "arm"
}
```

In which droneId is the previously defined id and cmd the action command. The
message follows the same format for the disarm, takeoff, land and return
commands. In the case of the goto command, additional parameters must be provided:

```json
{
    "droneId": "drone01",
    "mode": "action",
    "cmd": "goto",
    "lat": 40.633667,
    "lon": -8.660522,
    "alt": 15,
    "yaw": 0
}
```

Description:
- lat and lon : latitude and longitude, in degrees
- alt: altitude in meters above mean sea level
- yaw: yaw angle in degrees (0 is North, positive is clockwise)

### Command offboard
This is the set of offboard commands that are allowed:

- start (start offboard mode)
- stop (stop offboard mode)
- position_ned (set position in North-East-Down coordinates and yaw)
- velocity_ned (set velocity in North-East-Down coordinates and yaw)
- velocity_body (set velocity in Forward-Right-Down coordinates and yaw)
- attitude (set attitude in roll, pitch, yaw and thrust)
- attitude_rate (set attitude rate in roll, pitch, yaw and thrust)
- actuator_control

Offboard commands will only be executed after sending a “start” message. The “stop”
 message should be sent to stop offboard mode. While the offboard mode is active,
 it’s not possible to send goto action commands, but all other action commands
 (land and return, for example) can still be sent and will override the offboard mode.

These are some examples of the commands:

**Start/stop:**
```json
{
    "droneId": "drone01",
    "mode": "offboard",
    "cmd": "start"
}
```

**Position/velocity ned**
```json
{
    "droneId": "drone01",
    "mode": "offboard",
    "cmd": "position_ned",
    "north": 1,
    "east": 2,
    "down": 0,
    "yaw": 30
}
```

- north/east/down: meters (position), meters/second (velocity)
- yaw: degrees (position), degrees/second (velocity)


**Velocity body**
```json
{
    "droneId": "drone01",
    "mode": "offboard",
    "cmd": "velocity_body",
    "forward": 5,
    "right": 0,
    "down": 0,
    "yaw": 30
}
```

* Forward/Right/Down: meters/second
* Yaw: degrees/second
 
**Attitude/attitude rate**
```json
{
    "droneId": "drone01",
    "mode": "offboard",
    "cmd": "attitude",
    "roll": 0,
    "pitch": 30,
    "yaw": 30,
    "thrust": 0.5
}
```

- Roll: degrees (attitude), degrees/second (attitude_rate), positive is right side down
- Pitch: degrees (attitude), degrees/second (attitude_rate), positive is nose up
- Yaw: degrees (attitude), degrees/second (attitude_rate), positive is clockwise
- Thrust: percentage from 0 to 1

**Actuator control**
```json
{
    "droneId": "drone01",
    "mode": "offboard",
    "cmd": "actuator_control",
    "group0": [0, -1, 1, 0.5, 0, -1, 1, 0.5],
    "group1": [0, -1, 1, 0.5, 0, -1, 1, 0.5]
}
```

### Telemetry and Status
Telemetry and state information can be received by subscribing to the /telem
(according to the chosen topic in the configs) topic. This ROS message is also of
the type std_msgs/String  and formatted in JSON. This is an example:


```json
{
    "armed" : false,
    "battery" : {
        "remaining_percent" : 0.98,
        "voltage" : 12.15000057
    },
    "droneId" : "drone01",
    "flight_mode" : "hold",
    "heading" : -0.2981671095,
    "healthFail" :  ["accelerometer"],
    "home":{
        "alt" : 487.9980164,
        "lat" : 47.3979674,
        "lon" : 8.5431335
    },
    "landed_state" : "on_ground",
    "position" : {
        "alt" : 9.0650005340576172,
        "lat" : 40.633870299999998,
        "lon" : -8.6602946000000003
    },
    "timestamp" : 1586998439212
}
```

Data types:
- armed: bool
- battery/remaining_percent -- float
- battery/voltage -- float
- droneId: string
- flight_mode: one string out of this list [“unknown”, “ready”, “takeoff”, “hold”, “mission”, “return_to_launch”, “land”, “offboard”, “follow_me”, “manual”, “altctl”, “posctl”, “acro”, “stabilized”, “rattitude”]
- heading: float
- healthFail: list with zero or more strings out of this list [“gyrometer”, “accelerometer”, “magnetometer”, “level”, “levelPos”, “globalPos”, “homePos”]
- landed_state: one string out of this list [“unknown”, “on_ground”, “in_air”, “taking_off”, “landing”]
- position/alt: float
- position/lat: float
- position/lon: float
- timestamp: long

Description:
- droneId - id of the drone sending this message
- timestamp - unix timestamp in milliseconds
- healthFail - presents the health checks that failed; empty list in case all checks have passed; otherwise, contains all the failing parameters
- Others:  the remaining parameters weren’t altered and are as defined on Mavsdk documentation

Regarding action  completion status,  it is published on the /status  topic. These messages
can either represent a component's or a command's state. The format is as follows:

```json
{
  "timestamp":1590512172217,
  "droneId":"drone01",
  "component":"system",
  "state":"connect"
}
```

```json
{
  "timestamp":1590512277423,
  "droneId":"drone01",
  "command":"arm",
  "state":"success"
}
```

```json
   {
     "timestamp":1590512277423,
     "droneId":"drone01",
     "command":"goto",
     "state":"start",
     "coords": {
       "alt" : 9.0650005340576172,
       "lat" : 40.633870299999998,
       "lon" : -8.6602946000000003
     }
   }
   ```

Currently, the only component is "system", but it could represent a sensor, for example. In
case the message regards the "system" component, these are the possible states:
- `connect`:  connected Pixhawk
- `disconnect`: disconnected Pixhawk
- `disarm`: drone was auto-disarmed
- `return`: drone was triggered to return to launch
- `stop_offboard`: stopped offboard mode after issuing an action command

When the message is reporting the status of a command, the state can be one of these:

- `start`: command started to run
- `stop`: command was stopped
- `cancel`: command was cancelled because another command was sent
- `finish`: command finished executing
- `queued`: go to command was queued
- `success`: command succeeded
- `failure`: command failed 

The commands won't display all of these states; the following table describes which states
are possible for each command: 

| Command          | start              | stop               | cancel             | finish             | queued             | success            | failure            |
| ---------------- | ------------------ | ------------------ | ------------------ | ------------------ | ------------------ | ------------------ | ------------------ |
| arm              |                    |                    |                    |                    |                    | :heavy_check_mark: | :heavy_check_mark: |
| disarm           |                    |                    |                    |                    |                    | :heavy_check_mark: | :heavy_check_mark: |
| takeoff          | :heavy_check_mark: |                    | :heavy_check_mark: | :heavy_check_mark: |                    |                    | :heavy_check_mark: |
| land             | :heavy_check_mark: |                    | :heavy_check_mark: | :heavy_check_mark: |                    |                    | :heavy_check_mark: |
| goto             | :heavy_check_mark: |                    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |                    | :heavy_check_mark: |
| return           | :heavy_check_mark: |                    | :heavy_check_mark: | :heavy_check_mark: |                    |                    | :heavy_check_mark: |
| start offboard   |                    |                    |                    |                    |                    | :heavy_check_mark: | :heavy_check_mark: |
| stop offboard    |                    |                    |                    |                    |                    | :heavy_check_mark: | :heavy_check_mark: |
| position NED     | :heavy_check_mark: |                    | :heavy_check_mark: | :heavy_check_mark: |                    |                    | :heavy_check_mark: |
| velocity NED     | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    | :heavy_check_mark: |
| velocity body    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    | :heavy_check_mark: |
| attitude         | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    | :heavy_check_mark: |
| attitude rate    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    | :heavy_check_mark: |
| actuator control | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |                    |                    |                    | :heavy_check_mark: |

If the reported command is a starting `takeoff` or `return`, the `altitude` is also included;
while if it is the case of a `goto`, there will always be a `coords` structure carrying the
target coordinates. The remaining commands don't send extra fields.

# Package structure
The drone's codebase is divided into different files:

* `drone_controller`: main program which initializes the ROS2 nodes
* `drone_utils`: utility class to wrap some reusable code
* `mavsdk_handler`: node that executes commands it receives with Mavsdk and publishes
telemetry status
* `status_tracker`: exposes methods to monitor system and command status
