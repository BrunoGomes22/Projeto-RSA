enable 'replacement', drainage: 0.0015

// Summon the first drone
drone = summon_drone()
// Run mapping algorithm
finish_map = run { map_area(drone, lat: 40.633027, lon: -8.660440) }
// Wait until the mapping mission is finished
wait finish_map
move drone to lat: drone.home.lat, lon: drone.home.lon, speed: 10
land drone

// Initialize drone
def summon_drone() {
    def drone = assign any
    arm drone
    takeoff drone, 10.meters
    return drone
}

def map_area(pos=null, drone) {
    def x = 100.meters
    def y = 75.meters
    def n_steps_x = 2
    def n_steps_y = 3
    def step_size_x = x / n_steps_x
    def step_size_y = y / n_steps_y
    move drone at 10.m/s to lat: pos.lat, lon: pos.lon
    //move drone, lat: 40.632359, lon: -8.660392, speed: 3
    (n_steps_y+1).times {traversal ->
        if (traversal > 0)
            move drone right by: traversal.even ? step_size_y : -step_size_y
        turn drone to traversal.even ? right : left
        n_steps_x.times {
            move drone at 10.m/s forward by: step_size_x
        }
    }
}
