/*
 * friends_remote_monitoring.groovy
 * Any drone can perform this mission, as long as the corresponding drone-side FRIENDS service is running.
 * Start remote monitoring task.
 */

drone = assign any
drone_params = [dmax_speed: 5,
                dmin_bat: 25]
mission_params = [mpath_algo: 'boustrophedon',
                  maoi: [drone.position, drone.right(30.meters), drone.forward(50.meters).right(30.meters), drone.forward(50.meters)],
                  mheight: drone.position.alt + 5,
                  mstart: drone.position,
                  mstop: drone.position,
                  mtype: 'monitoring']
sensor_params = [sdist: 5]

monitor = start monitoring with drone: drone_params, mission: mission_params, sensor: sensor_params using drone
wait task: monitor
land drone
