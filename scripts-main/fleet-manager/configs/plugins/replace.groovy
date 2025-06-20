def calcNecessaryBattery = { drone ->
    return drainage * drone.distance(drone.home)
}

def is_returning_home = { drone ->
    drone.cmd == home || (drone.cmd == move && drone.cmd.distance(drone.home) < 50.cm)
}

def main = { drone ->
    if (!is_returning_home(drone) && calcNecessaryBattery(drone) > drone.battery - min_battery)
        replace drone
}


plugin {
    id      'replacement'
    type    monitor
    input   min_battery: [float, 0.25],
            drainage: [float, 0.0002]
    callback main
}
