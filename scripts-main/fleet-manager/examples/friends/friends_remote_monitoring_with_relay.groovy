/*
 * friends_remote_monitoring_with_relay.groovy
 * Any drone can perform this mission, as long as the corresponding drone-side FRIENDS service is running.
 * Starts remote monitoring task and a relay drone will be summoned.
 */

enable 'relay', target_dist: 50.m, step: 4.m, alt_diff: 5.m, base_alt: 5.m, gs_coords: [lat: 40.63347, lon: -8.66028]

drone = assign 'drone01'
drone_params = [dmax_speed: 5,
                dmin_bat: 25]
mission_params = [mpath_algo: 'boustrophedon',
                  maoi: [drone.position, drone.right(50.meters), drone.forward(80.meters).right(50.meters), drone.forward(80.meters)],
                  mheight: drone.position.alt + 5,
                  mstart: drone.position,
                  mstop: drone.position,
                  mtype: 'monitoring']
sensor_params = [sdist: 15]

monitor = start monitoring with drone: drone_params, mission: mission_params, sensor: sensor_params using drone
wait task: monitor
land drone
