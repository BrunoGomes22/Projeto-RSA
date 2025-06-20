/*
 * follow.groovy
 * Two drones are required for this mission
 * Send one drone to map an area, traversing it in an "S" shape. The second drone
 * will follow the first one above it.
 */


// Initialize and takeoff drone
(droneX, droneY) = assign any, any
x_takeoff = 5.m
y_takeoff = 15.m
arm droneX
takeoff droneX, x_takeoff
alt_x = droneX.position.alt
finish_map = run { map_area(droneX) }
arm droneY
takeoff droneY, y_takeoff
alt_y = droneY.position.alt

while (droneY.position.distance(droneX.position) > (y_takeoff - x_takeoff) + 1) {
    run { move droneY, lat: droneX.position.lat, lon: droneX.position.lon, alt: alt_y, speed: 5.m/s }
    wait droneY
}
prev_target = null
while (finish_map.running) {
    if (droneX.cmd == move && droneX.cmd.target != prev_target) {
        prev_target = droneX.cmd.target
        run {move droneY, lat: droneX.cmd.target.lat, lon: droneX.cmd.target.lon, alt: alt_y, speed: 3.m/s}
    }
    wait droneX
}
(finish_x, finish_y) = run {send_home(droneX, alt_x)}, {send_home(droneY, alt_y)}
wait finish_x, finish_y

def send_home(drone, alt) {
    move drone, lat: drone.home.lat, lon: drone.home.lon, alt: alt
    land drone
}

def map_area(drone) {
    // Define grid
    def x = 25.meters
    def y = 20.meters
    def n_steps_x = 1
    def n_steps_y = 3
    def step_size_x = x / n_steps_x
    def step_size_y = y / n_steps_y
    def alt = drone.position.alt
    def target

    move drone, lat: 40.632953, lon: -8.660074, alt: alt, speed: 3
    wait drone
    turn drone, south

    (n_steps_y+1).times {traversal ->
        if (traversal > 0) {
            target = drone.right(traversal.even ? step_size_y : -step_size_y)
            move drone, lat: target.lat, lon: target.lon, alt: alt, speed: 3.m/s
        }
        turn drone, traversal.even ? right : left
        n_steps_x.times {
            target = drone.forward(step_size_x)
            move drone, lat: target.lat, lon: target.lon, alt: alt, speed: 3.m/s
        }
    }
}