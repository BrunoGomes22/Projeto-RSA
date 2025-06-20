/*
 * multidrone_sync.groovy
 * Two drones are required to perform this mission: 'drone01' and 'drone02'.
 * Each drone will move through a specific path, but the second drone only starts after
 * the first one finishes taking off. Demonstrates how to synchronize two independently
 * running subroutines.
 */

// Define subroutine
routine1 = {
    arm drone01
    takeoff drone01, 2.m
    // Signal after finishing takeoff
    channel.foo = true
    6.times {
        move drone01, left: 12.m, forward: 3.m
    }
    land drone01
}

// Define subroutine
routine2 = {
    // Wait until drone01's takeoff has finished
    channel.foo
    arm drone02
    takeoff drone02, 2.m
    6.times {
        move drone02, right: 30.m, forward: 5.m
    }
    land drone02
}

// Assign drones
drone01 = assign 'drone01'
drone02 = assign 'drone02'

// Run subroutines
(r1, r2) = run routine1, routine2
// Wait for both subroutines to end before concluding mission
wait r1, r2