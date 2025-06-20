/*
 * replacement.groovy
 * Multiple drones may be required to perform this mission.
 * Send one drone to map an area and replace it with another drone after 1 minute.
 * The second drone will continue the mapping from where the previous one stopped.
 */

// Initialize and takeoff drone
enable 'replacement', min_battery: 0.7, drainage: 0.0002 // 0.02% battery drained for moving one meter; will drop 1% after moving 50 meters
drone = assign any
arm drone
takeoff drone, 15.m
move drone to lat: 40.631903529127875, lon: -8.660843617402127
turn drone to 225.deg
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
