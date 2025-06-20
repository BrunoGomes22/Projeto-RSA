gs_coords = [lat: 40.634125, lon: -8.660336]
enable 'relay', target_dist: 50.m, step: 4.m, alt_diff: 5.m, base_alt: 5.m, gs_coords: gs_coords
drone = assign 'drone01'
arm drone
takeoff drone
map_area(drone, 50.m, 50.m, 1, 3, [lat: 40.632963, lon: -8.660440, speed: 3.m/s])
move drone at 3.m/s to lat: drone.home.lat, lon: drone.home.lon
land drone

// Area mapping algorithm, explained in mapping.groovy example
def map_area(drone, x, y, steps_x, steps_y, init_pos=null) {
    def step_sz_x = x / steps_x
    def step_sz_y = y / steps_y

    if (init_pos != null)
        move drone to init_pos

    (steps_y+1).times { traversal ->
        if (traversal > 0)
            move drone right by: traversal.even ? step_sz_y : -step_sz_y
        turn drone to traversal.even ? right : left
        steps_x.times {
            move drone forward by: step_sz_x
        }
    }
}