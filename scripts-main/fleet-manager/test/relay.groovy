/*
 * relay.groovy
 * Multiple drones may be required to perform this mission.
 * Send one drone to map an area and recursively dispatch relay drones to
 * maintain connectivity to the groundstation.
 */

// Groundstation location
gs_coords = [lat: 40.633874, lon: -8.660311]
// Target distance to maintain between nodes
target_dist = 25.meters
// Approximate distance tolerance
step = 2.meters
// Current number of drones in air (used to set flying altitude)
n_drones = 0
being_followed = []
pending_drones = [:]

// Summon the first drone
drone = summon_drone('drone01')
// Run mapping algorithm
finish_map = run { map_area(drone) }
while (finish_map.running) {
    // Dispatch relay drone if getting far from groundstation
    if (check_backup(drone))
        break
    wait drone
}
// Wait until the mapping mission is finished
wait finish_map


// Drone following algorithm
def follow(other) {
    def dist, gs_dist, pos, followed_gs_dist
    // Dispatch drone
    println "BEFORE dist from gs to ${other.id}: ${other.position.distance(gs_coords.lat, gs_coords.lon)}"
    def self = summon_drone(other.position)
    def curr_target = self.position

    println "AFTER dist to ${other.id}: ${other.position.distance(self.position)}"
    // While the followed drone is still running
    while(other.state == active || other.state == hold) {
        // Ideal position where this drone should be placed (within the defined range, in line with gs)
        def other_gs_dist = other.position.distance(gs_coords.lat, gs_coords.lon)
        def cmd_gs_dist = other.cmd == move ? other.cmd.target.distance(gs_coords.lat, gs_coords.lon) : null
        pos = other.position.target(target_dist, other.position.bearing(gs_coords))
        if (pos.distance(other.position) > self.position.distance(other.position)) {
            if (other.cmd != move)
                pos = self.position
            else if (cmd_gs_dist > other_gs_dist) {
                if (other_gs_dist > target_dist*0.9)
                    pos = other.position.target(target_dist * 0.9, other.position.bearing(gs_coords))
                else
                    pos = self.position
            }
        }

        // Distance between the ideal position and where the drone is/is currently moving to
        dist = pos.distance(curr_target)

        // Free this drone if the followed drone is close to groundstation
        if (other_gs_dist < target_dist && other.cmd == move && other_gs_dist - cmd_gs_dist > step) {
            println "$self.id - landing"
            println "$other_gs_dist < $target_dist && $other_gs_dist - $cmd_gs_dist > step}"
            cancel self
            being_followed.remove(other.id)
            break
        }

        // If the distance between the target and ideal position is greater than step size,
        // move the drone to the ideal position and set it as target
        if (dist > step) {
            println "dist from ${self.id} to ${other.id}: ${other.position.distance(self.position)}"
            curr_target = pos
            def bearing = self.position.bearing(other.position)
            if (self.position.distance(other.position) < curr_target.distance(other.position))
                bearing += 180
            run { move self, lat: curr_target.lat, lon: curr_target.lon, yaw: bearing, speed: other.speed < 0.2 ? 0.5 : min(other.speed + 2, 10) }
        }

        // If the current drone isn't being followed and is getting far from the groundstation, dispatch another one
        if (other.cmd == move && other_gs_dist < cmd_gs_dist) {
            wait self
            check_backup(self)
        }
        wait other
    }
    // Return this drone to home position
    pending_drones[self.id] = [self, run { move_home(self) }]
}

def move_home(drone) {
    n_drones--
    move drone, lat: drone.home.lat, lon: drone.home.lon, speed: 3
    land drone
    pending_drones.remove(drone.id)
    revoke drone
}

def check_backup(drone) {
    def gs_dist = drone.position.distance(gs_coords.lat, gs_coords.lon)

    if (!being_followed.contains(drone.id) && drone.cmd == move && drone.speed > 0.2) {
        def cmd = drone.cmd.target
        def cmd_dist = drone.position.distance(cmd)
        def cmd_gs_dist = cmd.distance(gs_coords.lat, gs_coords.lon)

        if (cmd_gs_dist > gs_dist) {
            def tmp
            if (cmd_dist < target_dist/3)
                tmp = target_dist*0.8
            else
                tmp = target_dist*0.6

            if (gs_dist > tmp) {
                println "$drone.id - requesting backup"
                run { follow(drone) }
                being_followed += drone.id
                return true
            }
        }
    }
    return false
}

// Initialize drone
def summon_drone(requirement) {
    def drone
    if (pending_drones.isEmpty()) {
        drone = assign requirement
        arm drone
        println "Summon drone $drone.id"
    } else {
        def tmp = pending_drones.remove(pending_drones.keySet()[0])
        stop tmp[1]
        drone = tmp[0]
        println "Reassign drone $drone.id"
    }
    // Different takeoff altitudes to avoid colliding
    takeoff drone, (15.m - n_drones*5)
    n_drones++
    return drone
}


def map_area(drone) {
    def alt = drone.position.alt
    move drone, lat: 40.633904, lon: -8.660550, alt: alt, speed: 1.m/s
    move drone, lat: 40.634240, lon: -8.660412, alt: alt, speed: 1.m/s
    move drone, lat: 40.634035, lon: -8.660757, alt: alt, speed: 1.m/s
    move drone, lat: 40.633867, lon: -8.660780, alt: alt, speed: 1.m/s
    move drone, lat: 40.633951, lon: -8.660684, alt: alt, speed: 1.m/s
    move drone, lat: drone.home.lat, lon: drone.home.lon, speed: 1.m/s
    land drone
}