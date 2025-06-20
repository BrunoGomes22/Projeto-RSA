/*
 * mapping.groovy
 * Any drone can perform this mission
 * Send one drone to map an area, traversing it in an "S" shape. The mapped area will be
 * the first quadrant with the drone's initial position as the origin.
 */

// Initialize and takeoff drone
drone = assign any
arm drone
takeoff drone, 15.m
move drone to lat: 40.633869406546665, lon: -8.660419196426341
turn drone to -90.deg
println "Ready to start mapping"
map_area(drone, 30, 25, 2, 3, 3)
move drone, lat: drone.home.lat, lon: drone.home.lon, alt: 15.m, speed: 3
land drone


def map_area(drone, x, y, steps_x, steps_y, speed) {
    def step_sz_x = x / steps_x
    def step_sz_y = y / steps_y

    (steps_y+1).times { traversal ->
        if (traversal > 0)
            move drone at speed right by: traversal.even ? step_sz_y : -step_sz_y
        turn drone to traversal.even ? right : left
        steps_x.times {
            move drone at speed forward by: step_sz_x
        }
    }
}