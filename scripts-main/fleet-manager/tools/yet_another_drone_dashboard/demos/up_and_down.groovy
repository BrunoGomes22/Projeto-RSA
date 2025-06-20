droneA = assign any
droneB = assign any
/*
startA = run{init_drone(droneA)}
startB = run{init_drone(droneB)}
wait tasks: [startA, startB]
moveA = run {up_and_down(droneA)}
moveB = run {down_and_up(droneB)}x
wait tasks: [moveA, moveB]
wait 2.s
moveA = run {down_and_up(droneA)}
moveB = run {up_and_down(droneB)}
wait tasks: [moveA, moveB]
wait 2.s
finishA = run{ finish(droneA) }
finishB = run{ finish(droneB) }
wait tasks: [finishA, finishB]*/

startA = run{init_drone(droneA)}
startB = run{init_drone(droneB)}
wait_for(startA, startB)
a = run {move_up(droneA)}
b = run {move_down(droneB)}
wait_for(a, b)
a = run {move_down(droneA)}
b = run {move_up(droneB)}
wait_for(a, b)
a = run {move_down(droneA)}
b = run {move_up(droneB)}
wait_for(a, b)
a = run {move_up(droneA)}
b = run {move_down(droneB)}
wait_for(a, b)
finishA = run{ finish(droneA) }
finishB = run{ finish(droneB) }
wait tasks: [finishA, finishB]

def wait_for(taskA, taskB) {
    wait tasks: [taskA, taskB]
    wait 2.s
}

def init_drone(drone) {
    arm drone
    takeoff drone, 5.m
}

def move_up(drone) {
    move drone up by: 2.5.m
}

def move_down(drone) {
    move drone down by: 2.5.m
}

def up_and_down(drone) {
    move drone up by: 2.5.m
    wait 2.s
    move drone down by: 2.5.m
}

def down_and_up(drone) {
    move drone down by: 2.5.m
    wait 2.s
    move drone up by: 2.5.m
}

def finish(drone) {
    land drone
    wait drone
    if (drone.armed)
        disarm drone
}