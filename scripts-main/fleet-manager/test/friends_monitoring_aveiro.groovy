drone = assign any
drone_params = [dmax_speed: 5,
                dminBat: 25]
mission_params = [mpath_algo: 'boustrophedon',
                  maoi: [40.63368892186809, -8.66088045443767,
                         40.633412491007185, -8.660871432597888,
                         40.6334193376058, -8.660498153976883,
                         40.63368977768941, -8.660513942196504],
                  mheight: 5,
                  mstart: drone.position,
                  mstop: drone.position,
                  mtype: 'monitoring']
sensor_params = [sdist: 10]
arm drone
takeoff drone, 5.5.m
monitor = start monitoring with drone: drone_params, mission: mission_params, sensor: sensor_params using drone
wait task: monitor
wait 10.s
home drone
