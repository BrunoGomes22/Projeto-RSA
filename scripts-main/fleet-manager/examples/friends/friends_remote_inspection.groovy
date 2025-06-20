/*
 * friends_remote_inspection.groovy
 * Any drone can perform this mission, as long as the corresponding drone-side FRIENDS service is running.
 * Start remote inspection task.
 */

drone = assign any
drone_params = [dmax_speed: 5,
                dmin_bat: 25]
mission_params = [mpath_algo: 'fast',
                  maoi: [40.6310321, -8.6609289,
                         40.6310732, -8.660825,
                         40.6309723, -8.660982],
                  mheight: drone.position.alt + 5,
                  mstart: drone.forward(2.m),
                  mstop: drone.position,
                  mtype: 'inspection']
sensor_params = [sdist: 0, sgrab_time: 25]

inspect = start inspection with drone: drone_params, mission: mission_params, sensor: sensor_params using drone
wait task: inspect
land drone
