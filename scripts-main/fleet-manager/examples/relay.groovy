/*
 * relay.groovy
 * Multiple drones may be required to perform this mission.
 * Send one drone to map an area and recursively dispatch relay drones to
 * maintain connectivity to the groundstation.
 */

// Groundstation location
gs_coords = [lat: 40.634125, lon: -8.660336] // [lat: 40.634024, lon: -8.660350]
// Target distance to maintain between nodes
target_dist = 50.meters
// Approximate distance tolerance
step = 4.meters
// Current number of drones in air (used to set flying altitude)
n_drones = 0
// Relay drones that are already being followed
being_followed = []
pending_drones = [:]

// Summon the first drone
main_drone = dispatch_drone('drone01')
// Run mapping algorithm
mapping = run { map_area(main_drone) }
while (mapping.running) {
    // Dispatch relay drone if getting far from groundstation
    monitor_gs_distance(main_drone)
    wait main_drone
}
move_home(main_drone)

def monitor_gs_distance(drone) {
    if (!being_followed.contains(drone.id) && requires_relay(drone)) {
        println "$drone.id - requesting backup"
        run { follow(drone) }
        being_followed += drone.id
    }
}

def requires_relay(drone) {
    drone.cmd == move && drone.cmd.distance(gs_coords) > target_dist && drone.distance(gs_coords) + 10*drone.speed > target_dist*0.75
}

// Drone following algorithm
def follow(other) {
    def new_target
    // Dispatch drone
    def self = dispatch_drone(other.position)
    def curr_target = self.position
    println "${self.id} ready to serve - dist ${self.distance(other.position)}"

    // While the followed drone is still running
    while(other.state == active || other.state == hold) {
        // Free this drone if the followed drone is close to groundstation
        if (is_returning_to_gs(other)) {
            println "$self.id - landing"
            cancel self
            being_followed.remove(other.id)
            break
        }

        // Ideal position where this drone should be placed (within the defined range, in line with gs)
        new_target = calc_new_target(other, self)

        // If the distance between the target and ideal position is greater than step size,
        // move the drone to the ideal position and set it as target
        if (new_target.distance(curr_target) > step) {
            curr_target = new_target
            run { move self at calc_new_speed(other) to lat: curr_target.lat, lon: curr_target.lon, yaw: calc_new_bearing(other, self, curr_target)}
        }

        // If the current drone isn't being followed and is getting far from the groundstation, dispatch another one
        if (is_moving_further(other)) {
            wait self
            monitor_gs_distance(self)
        }
        wait other
    }
    // Return this drone to home position
    revoke_relay_drone(self)
}

def is_moving_further(drone) {
    return drone.cmd == move && drone.distance(gs_coords) < drone.cmd.distance(gs_coords)
}

def is_returning_to_gs(drone) {
    drone.distance(gs_coords) < target_dist && !is_moving_further(drone)
}

def calc_new_target(other, self) {
    def new_target, dist = self.distance(other.position)
    if (dist < target_dist && is_moving_further(other)) {
        if (dist > target_dist / 2)
            new_target = other.target(other.distance(gs_coords) / 2, other.bearing(gs_coords))
        else
            new_target = self.position
    } else
        new_target = other.target(target_dist, other.bearing(gs_coords))
    new_target
}

def calc_new_bearing(other, self, target) {
    def bearing = self.bearing(other.position)
    if (self.distance(gs_coords) > target.distance(gs_coords))
        bearing += 180
    bearing
}

def calc_new_speed(other) {
    min(other.speed + 2, 10)
}

def move_home(drone) {
    n_drones--
    move drone at 3.m/s to lat: drone.home.lat, lon: drone.home.lon
    land drone
    pending_drones.remove(drone.id)
    revoke drone
}

def revoke_relay_drone(drone) {
    pending_drones[drone.id] = [drone: drone, landing: run { move_home(drone) }]
}

def reassign_relay_drone() {
    def pending_data = pending_drones.remove(pending_drones.keySet()[0])
    def drone = pending_data.drone
    stop pending_data.landing
    if (drone.armed) {
        def alt_diff = n_drones.even ? 10.meters : 5.meter - (drone.position.alt - drone.home.alt)
        if (alt_diff > 1.meter)
            takeoff drone, alt_diff
    } else {
        arm drone
        takeoff drone, n_drones.even ? 10.meters : 5.meters
    }
    drone
}

// Initialize drone
def dispatch_drone(requirement) {
    def drone
    if (pending_drones.isEmpty()) {
        drone = assign requirement
        arm drone
        println "Dispatch drone $drone.id"
        takeoff drone, n_drones.even ? 10.meters : 5.meters
    } else {
        drone = reassign_relay_drone()
        println "Reassign drone $drone.id"
    }
    n_drones++
    return drone
}

// Area mapping algorithm, explained in mapping.groovy example
def map_area(drone) {
    def x = 50.meters
    def y = 50.meters
    def n_steps_x = 1
    def n_steps_y = 3
    def step_size_x = x / n_steps_x
    def step_size_y = y / n_steps_y
    move drone at 3.m/s to lat: 40.632963, lon: -8.660440
    //move drone, lat: 40.632359, lon: -8.660392, speed: 3
    (n_steps_y+1).times {traversal ->
        if (traversal > 0)
            move drone right by: traversal.even ? step_size_y : -step_size_y
        turn drone to traversal.even ? right : left
        n_steps_x.times {
            move drone forward by: step_size_x
        }
    }
}