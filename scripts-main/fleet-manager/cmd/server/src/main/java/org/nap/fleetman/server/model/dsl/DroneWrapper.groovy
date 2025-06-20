package org.nap.fleetman.server.model.dsl

import org.nap.fleetman.server.mission.WrapperManager
import org.nap.fleetman.server.mission.dsl.PositionCategory
import org.nap.fleetman.server.model.drone.Command
import org.nap.fleetman.server.model.drone.Drone
import org.nap.fleetman.server.model.drone.DroneState
import org.nap.fleetman.server.model.telemetry.Position
import org.nap.fleetman.server.model.telemetry.Telemetry
import org.nap.fleetman.server.remote.RemoteTask

class DroneWrapper implements Cloneable {
    private String id
    Boolean armed
    DroneState state
    Command cmd
    Position position
    Position home
    Float battery
    Double heading
    Float speed
    Float height
    Long timestamp
    Map params
    RemoteTask task
    boolean revoked
    boolean isBeingReplaced

    DroneWrapper(String id) {
        this.id = id
        this.revoked = false
        this.isBeingReplaced = false
    }

    String getId() {
        id
    }

    void setId(String id) {
        def notCalledByWrapperMan = Thread.currentThread().getStackTrace().find { it.className == WrapperManager.name } == null
        if (notCalledByWrapperMan && Thread.currentThread().name.split("_").size() > 1)
            throw new SecurityException()
        this.id = id
    }

    void updateTelem(Drone drone, Telemetry telem) {
        armed = telem.armed
        position = telem.getPosition()
        speed = telem.getSpeed()
        home = telem.getHome()
        battery = telem.getBattery().getPercentage()
        heading = telem.getHeading()
        height = telem.getHeight()
        timestamp = telem.getTimestamp()
        state = drone.state
        cmd = drone.currentCommand
    }

    String toString() {
        "droneId: $id, state: $state, ${cmd == null ? "cmd: null" : "cmd type: $cmd.type, cmd target: $cmd.target"}, " +
                "position: $position, home: $home, battery: $battery, speed: $speed, heading: $heading, timestamp: $timestamp"
    }

    def forward(Distance dist) {
        def pos = PositionCategory.target(position, dist, heading)
        [lat: pos.lat, lon: pos.lon, alt: pos.alt, bearing: heading]
    }

    def backward(Distance dist) {
        def pos = PositionCategory.target(position, dist, heading + 180)
        [lat: pos.lat, lon: pos.lon, alt: pos.alt, bearing: heading - 180]
    }

    def left(Distance dist) {
        def pos = PositionCategory.target(position, dist, heading - 90)
        [lat: pos.lat, lon: pos.lon, alt: pos.alt, bearing: heading - 90]
    }

    def right(Distance dist) {
        def pos = PositionCategory.target(position, dist, heading + 90)
        [lat: pos.lat, lon: pos.lon, alt: pos.alt, bearing: heading - 90]
    }

    def revoke() {
        revoked = true
    }
}
