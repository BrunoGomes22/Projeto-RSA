drone = assign 'drone02'
arm drone
takeoff drone, 4.m
4.times {
    move drone at 2.m/s right by: 4.m
}
land drone
wait drone
if (drone.armed)
    disarm drone
