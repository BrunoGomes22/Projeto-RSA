/*
 * replacement.groovy
 * Multiple drones may be required to perform this mission.
 * Send one drone to map an area and replace it with another drone after 1 minute.
 * The second drone will continue the mapping from where the previous one stopped.
 */

// Initialize variables
drone = null
start_time = null

// Summon the first drone
summon_drone()
// Start mapping the area and wait until it is fully covered
finish_map = run {map_area(60.m, 40.m, 3, 3)}
wait finish_map
// Send current drone to launch coordinates and land
home drone


// Initialize drone
def summon_drone(pos=null) {
    drone = assign any
    arm drone
    takeoff drone, 3.m
    // If replacing another drone, move to where it left off
    if (pos != null)
        move drone, pos
}

// Send move command after verifying if the drone needs to be replaced
def move_or_replace(movement) {
    // Replace drone according to battery
    if (drone.battery < 0.4) {
        def curr_pos = [lat: drone.position.lat, lon: drone.position.lon, alt: drone.position.alt, yaw: drone.heading]
        // Send current drone to launch coordinates and land 
        run {home drone}
        revoke drone
        // Summon new drone
        summon_drone(curr_pos)
    }
    move drone, movement
}

// Area mapping algorithm, explained in mapping.groovy example

def map_area(x, y, steps_x, steps_y) {
    def step_sz_x = x / steps_x
    def step_sz_y = y / steps_y

    (steps_y+1).times { traversal ->
        if (traversal > 0)
            move_or_replace right: traversal.even ? step_sz_y : -step_sz_y
        turn drone, traversal.even ? right : left
        steps_x.times {
            move_or_replace forward: step_sz_x
        }
    }
}