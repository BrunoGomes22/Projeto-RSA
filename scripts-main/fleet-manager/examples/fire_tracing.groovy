/*
 * fire_tracing.groovy
 * Requires a drone with a temperature sensor to perform this mission.
 * Starts mapping an area and if an increase in temperature is detected,
 * traces the perimeter of the fire.
 */

// Initialize drone
drone = assign 'temperature'
arm drone
takeoff drone, 3.m

start_pos = null

map = run {map_area(drone)}
while (map.running) {
    println "${sensor.(drone.id).temperature}"
    if (sensor.(drone.id).temperature > 37) {
        start_pos = drone.position
        stop map
        trace(drone)
        break
    }
    wait drone
}
home drone

def trace(drone) {
    def error = null, error_sum = 0, deg
    boolean first = true
    while (true) {
        (error, deg) = pid(error, error_sum)
        error_sum += error
        wait drone
        move drone at 10.m/s to drone.target(4.meters, drone.heading + deg)
        if (first) {
            if (drone.distance(start_pos) > 7.meters)
                first = false
        } else if (drone.distance(start_pos) < 5.meters)
            break
    }
}

// Calculate PID
def pid(last_value = null, error_sum = null) {
    def target_temperature = 40
    float error = sensor.(drone.id).temperature - target_temperature
    def a = 8, b = 10, g = 0.01, angle
    if (last_value == null)
        angle = a * error
    else {
        angle = (a * error) + (b * (error - last_value)) + (g * (error_sum + error))
    }

    [error, angle]
}

// Area mapping algorithm, explained in mapping.groovy example
def map_area(drone) {
    def x = 30.meters
    def y = 40.meters
    def n_steps_x = 2
    def n_steps_y = 3
    def step_size_x = x / n_steps_x
    def step_size_y = y / n_steps_y

    (n_steps_y+1).times {traversal ->
        if (traversal > 0)
            move drone, right: traversal.even ? step_size_y : -step_size_y
        turn drone, traversal.even ? right : left
        n_steps_x.times {
            move drone, forward: step_size_x
        }
    }
}