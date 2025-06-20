/*
 * multidrone_basic_subroutine.groovy
 * Two drones are required to perform this mission: one has to be 'drone01'; the second drone may be any drone.
 * Move both drones forward simultaneously, demonstrating how to wait for launched subroutines.
 */

// Define behavior
move_forward = { d ->
    arm d
    takeoff d
    move d, forward: 3.m
    land d
}

// Assign two drones, requesting droneX to be 'drone01' specifically
(droneX, droneY) = assign 'drone01', any

// Run the subroutine for each drone
(var01, var02) = run { move_forward(droneX) }, { move_forward(droneY) }
// Wait for subroutines to be finished
wait var01, var02
