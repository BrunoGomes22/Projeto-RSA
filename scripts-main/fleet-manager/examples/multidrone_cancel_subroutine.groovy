/*
 * multidrone_cancel_subroutine.groovy
 * Two drones are required to perform this mission: 'drone01' and 'drone02'.
 * Both drones will repeatedly move to the left, drawing a square. The first drone will stop
 * and land after 30 seconds, and the second after 45 seconds. Demonstrates how to run
 * independent subroutines for each drone and how to stop a running subroutine.
 */

// Define subroutines
square = { drone ->
    arm drone
    takeoff drone, 3
    while(true) {
        move drone, left: 10.m
    }
}

// Assign two drones
(drone01, drone02) = assign 'drone01', 'drone02'

// Run the two subroutines
(s1, s2) = run { square(drone01) }, { square(drone02) }

// Mark starting timestamp in milliseconds
startTime = now()
drone01Finished = false

while (true) {
    // Stop "square2" subroutine execution after 45 seconds
    if (now() - startTime > 45000) {
        stop s2
        break
        // Stop "square1" subroutine execution after 30 seconds
    } else if (!drone01Finished && now() - startTime > 30000) {
        stop s1
        // A subroutine was launched because otherwise execution would halt til
        // the command was completed
        run { land drone01 }
        drone01Finished = true
    } else {
        wait drone02
    }
}

// Land drone02 before concluding mission
land drone02
