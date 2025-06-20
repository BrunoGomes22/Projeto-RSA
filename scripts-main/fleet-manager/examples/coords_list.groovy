/*
 * coords_list.groovy
 * Drone with id 'drone01' is specifically requested to perform this mission.
 * Send drone to 3 different waypoints, demonstrating how to iterate over a list
 * of coordinates and sending the move command in different ways.
 */

// Assign drone01 to variable "drone"
drone = assign 'drone01'

// Define a set of coordinates
coords = [
        // Move at a speed of 12m/s, no yaw is provided so the drone will face the waypoint
        [lat: 40.63392, lon: -8.6605474, speed: 12.m/s],
        // Default speed and yaw of 45º
        [lat: 40.63407, lon: -8.6606172, yaw: 45.deg],
        // Speed and yaw will have the default values
        [lat: 40.63398, lon: -8.6607840]
]

// Altitude ASML when landed
ground_alt = drone.position.alt
// Arm drone
arm drone
// Takeoff 5 meters
takeoff drone, 5.meters
// Altitude ASML after takeoff
takeoff_alt = drone.position.alt

// Iterate over coordinates
coords.each { coord ->
    // Send the map and append the takeoff altitude
    move drone, coord + [alt: takeoff_alt]
}

// Iterate over coordinates
coords.size().times { i ->
    // Send the coordinate values directly
    move drone, lat: coords[i].lat, lon: coords[i].lon, speed: coords[i].speed, yaw: coords[i].yaw
}

// Iterate over coordinates
for (int i=0; i < coords.size(); i++) {
    // Send the map and append takeoff altitude, going up one meter at each point
    move drone, coords[i] + [alt: drone.position.alt + 1]
}

// Return to takeoff location
home drone