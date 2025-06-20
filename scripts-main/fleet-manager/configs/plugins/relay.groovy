def is_moving_further = { drone ->
    return drone.cmd == move && drone.distance(gs_coords) < drone.cmd.distance(gs_coords)
}

def is_returning_to_gs = { drone ->
    drone.distance(gs_coords) < target_dist && !is_moving_further(drone)
}

def calc_new_target = { other, self ->
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

def calc_new_bearing = { other, self, target ->
    def bearing = self.bearing(other.position)
    if (self.distance(gs_coords) > target.distance(gs_coords))
        bearing += 180
    bearing
}

def calc_new_speed = { other ->
    min(other.speed + 2, 10)
}

def move_home = { drone ->
    n_drones--
    move drone to lat: drone.home.lat, lon: drone.home.lon, speed: 3
    land drone
    pending_drones.remove(drone.id)
    revoke drone
}

def revoke_relay_drone = { drone ->
    pending_drones[drone.id] = [drone: drone, landing: run { move_home(drone) }]
}

def reassign_relay_drone = {
    def pending_data = pending_drones.remove(pending_drones.keySet()[0])
    def drone = pending_data.drone
    stop pending_data.landing
    if (drone.armed) {
        def alt_diff = (n_drones.even ? base_alt + alt_diff : base_alt) - (drone.position.alt - drone.home.alt)
        if (alt_diff > 1.meter)
            takeoff drone, alt_diff
    } else {
        arm drone
        takeoff drone, n_drones.even ? base_alt + alt_diff : base_alt
    }
    drone
}

// Initialize drone
def dispatch_drone = { requirement ->
    def drone
    if (pending_drones.isEmpty()) {
        drone = assign requirement
        arm drone
        println "Dispatch drone $drone.id"
        takeoff drone, n_drones.even ? base_alt + alt_diff : base_alt
    } else {
        drone = reassign_relay_drone()
        println "Reassign drone $drone.id"
    }
    n_drones++
    return drone
}

// Drone following algorithm
def follow = { other ->
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
            def new_speed = calc_new_speed(other)
            def new_bearing = calc_new_bearing(other, self, curr_target)
            run { move self at new_speed to lat: curr_target.lat, lon: curr_target.lon, yaw: new_bearing}
        }

        wait other
    }
    // Return this drone to home position
    revoke_relay_drone(self)
}

def requires_relay = { drone ->
    def takeoff_time = n_drones.even ? 15 : 10
    drone.cmd == move && drone.cmd.distance(gs_coords) > target_dist
            && drone.distance(gs_coords) + takeoff_time * drone.speed > target_dist * 0.75
}

def monitor_gs_distance = { drone ->
    if (!being_followed.contains(drone.id) && requires_relay(drone)) {
        println "$drone.id - requesting backup ${drone.distance(gs_coords)}"
        run { follow(drone) }
        being_followed += drone.id
    }
}

plugin {
    id 'relay'
    type    monitor
    input   target_dist: Distance,
            step: Distance,
            gs_coords: [Map, [lat: 40.633874, lon: -8.660311]],
            alt_diff: [Distance, 3.m],
            base_alt: [Distance, 5.m]
    vars    being_followed: [],
            n_drones: 0,
            pending_drones: [:]

    callback monitor_gs_distance
}
