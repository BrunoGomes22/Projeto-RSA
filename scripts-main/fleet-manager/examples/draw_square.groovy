/*
 * draw_square.groovy
 * Any drone can perform this mission.
 * Send drone to 3 different waypoints, demonstrating how to iterate over a list
 * Move the drone in a square shape, requesting that it moves forward and turns.
 */

// Assign any drone that is available
drone = assign any

arm drone
takeoff drone, 5.m
4.times {
    move drone, forward: 20.m, speed:5.m/s
    turn drone, 90.deg
}
land drone
