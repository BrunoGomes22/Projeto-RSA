/*
 * friends_remote_monitoring_multiple.groovy
 * Any two drones can perform this mission, as long as the corresponding drone-side FRIENDS service is running.
 * Start remote monitoring task, sending two different areas of interest for each drone.
 */

(droneA, droneB) = assign 'drone01', 'drone02'

drone_params = [dmax_speed: 5, dmin_bat: 25]
sensor_params = [sdist: 5]

taskA = run { launch_task(droneA, [droneA.position, droneA.right(20.meters), droneA.forward(15.meters).right(20.meters), droneA.forward(15.meters)]) }
taskB = run { launch_task(droneB, [droneA.forward(15.meters), droneA.forward(15.meters).right(20.meters), droneA.forward(30.meters).right(20.meters), droneA.forward(30.meters)]) }

wait tasks: [taskA, taskB]

def launch_task(drone, aoi) {
    def mission_params = [mpath_algo: 'boustrophedon',
                          maoi: aoi,
                          mheight: drone.position.alt + 7,
                          mstart: drone.position,
                          mstop: drone.position,
                          mtype: 'monitoring']
    def monitor = start monitoring with drone: drone_params, mission: mission_params, sensor: sensor_params using drone
    wait task: monitor
    home drone
}
