/*
 * friends_remote_monitoring_pause_resume.groovy
 * Any drone can perform this mission, as long as the corresponding drone-side FRIENDS service is running.
 * Start remote monitoring task and pause for 10 seconds after it reaches 20% completion.
 */

drone = assign any
drone_params = [dmax_speed: 5,
                dmin_bat: 25]
mission_params = [mpath_algo: 'boustrophedon',
                  maoi: [drone.position, drone.right(30.meters), drone.forward(40.meters).right(30.meters), drone.forward(40.meters)],
                  mheight: drone.position.alt + 5,
                  mstart: drone.position,
                  mstop: drone.position,
                  mtype: 'monitoring']
sensor_params = [sdist: 10]

start monitoring with drone: drone_params, mission: mission_params, sensor: sensor_params using drone
while (drone.task.progress < 50)
    wait telem: drone
pause monitoring using drone
wait 10.seconds
monitor = resume monitoring using drone
wait monitor
home drone
