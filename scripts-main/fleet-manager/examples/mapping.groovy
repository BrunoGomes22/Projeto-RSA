/*
 * mapping.groovy
 * Any drone can perform this mission
 * Send one drone to map an area, traversing it in an "S" shape. The mapped area will be
 * the first quadrant with the drone's initial position as the origin.
 */

// Define grid dimensions, in which y is headed forward
x = 30.meters
y = 20.meters
// How many steps for the drone to complete a horizontal and vertical traversal
n_steps_x = 1
n_steps_y = 2
// Horizontal and vertical step sizes
step_size_x = x / n_steps_x
step_size_y = y / n_steps_y

// Initialize and takeoff drone
drone = assign any
arm drone
takeoff drone, 3.m

// The area is horizontally traversed n+1 times (horizontal borders always covered)
(n_steps_y+1).times {traversal ->
    // After the first horizontal traversal (traversal 0), move vertically, going left before
    // an odd traversal number and right before an even traversal number
    if (traversal > 0)
        move drone, right: traversal.even ? step_size_y : -step_size_y

    // Turn right on even traversals (left-to-right traversal) and left on odd ones (right-to-left traversal)
    turn drone, traversal.even ? right : left

    // For each horizontal step, move the corresponding width
    n_steps_x.times {
        move drone, forward: step_size_x
    }
}

// Return to launch position
home drone
