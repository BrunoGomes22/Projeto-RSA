/*
 * single_drone_to_location.groovy
 * Sends one drone to a target location and returns home.
 */

// Assign drone01 to variable "drone"
drone = assign 'drone01'

// Define the target coordinate
target = [lat: 40.63450, lon: -8.660800]

// Arm drone
arm drone
// Takeoff 5 meters
takeoff drone, 5.meters
// Altitude after takeoff
takeoff_alt = drone.position.alt

// Move to target at takeoff altitude
move drone, lat: target.lat, lon: target.lon, alt: takeoff_alt

// Return to takeoff location
home drone