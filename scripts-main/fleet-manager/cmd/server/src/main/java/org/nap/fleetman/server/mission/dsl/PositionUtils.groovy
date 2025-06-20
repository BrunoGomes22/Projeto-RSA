package org.nap.fleetman.server.mission.dsl

import org.nap.fleetman.server.model.drone.Command
import org.nap.fleetman.server.model.dsl.Distance
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.model.telemetry.Position
import org.nap.fleetman.server.utils.CoordUtils

class PositionUtils {
    Distance distance(Position src, Position dst) {
        PositionCategory.distance(src, dst)
    }

    Distance distance(Position src, double lat, double lon) {
        PositionCategory.distance(src, lat, lon)
    }

    Distance distance(Position src, Map dst) {
        PositionCategory.distance(src, dst)
    }

    Distance distance(Position src, DroneWrapper dst) {
        PositionCategory.distance(src, dst)
    }

    Distance distance(Map src, Position dst) {
        PositionCategory.distance(src, dst)
    }

    Distance distance(Map src, DroneWrapper dst) {
        PositionCategory.distance(src, dst)
    }

    Distance distance(Map src, double lat, double lon) {
        PositionCategory.distance(src, lat, lon)
    }

    Distance distance(Map src, Map dst) {
        PositionCategory.distance(src, dst)
    }

    Distance distance(DroneWrapper src, DroneWrapper dst) {
        PositionCategory.distance(src, dst)
    }

    Distance distance(DroneWrapper src, Position dst) {
        PositionCategory.distance(src, dst)
    }

    Distance distance(DroneWrapper src, Map dst) {
        PositionCategory.distance(src, dst)
    }

    Distance distance(DroneWrapper src, double lat, double lon) {
        PositionCategory.distance(src, lat, lon)
    }

    Position target(Position src, Number distance, double bearing) {
        PositionCategory.target(src, distance, bearing)
    }

    Position target(Position src, Distance distance, double bearing) {
        PositionCategory.target(src, distance, bearing)
    }

    Position target(DroneWrapper src, Number distance, double bearing) {
        PositionCategory.target(src, distance, bearing)
    }

    Position target(DroneWrapper src, Distance distance, double bearing) {
        PositionCategory.target(src, distance, bearing)
    }

    def target(Map src, Number distance, double bearing) {
        PositionCategory.target(src, distance, bearing)
    }

    def target(Map src, Distance distance, double bearing) {
        PositionCategory.target(src, distance, bearing)
    }

    double bearing(Position src, double lat, double lon) {
        PositionCategory.bearing(src, lat, lon)
    }

    double bearing(Position src, Position dst) {
        PositionCategory.bearing(src, dst)
    }

    double bearing(Position src, DroneWrapper dst) {
        PositionCategory.bearing(src, dst)
    }

    double bearing(Position src, Map dst) {
        PositionCategory.bearing(src, dst)
    }

    double bearing(DroneWrapper src, DroneWrapper dst) {
        PositionCategory.bearing(src, dst)
    }

    double bearing(DroneWrapper src, Position dst) {
        PositionCategory.bearing(src, dst)
    }

    double bearing(DroneWrapper src, Map dst) {
        PositionCategory.bearing(src, dst)
    }

    double bearing(DroneWrapper src, double lat, double lon) {
        PositionCategory.bearing(src, lat, lon)
    }

    double bearing(Map src, double lat, double lon) {
        PositionCategory.bearing(src, lat, lon)
    }

    double bearing(Map src, Position dst) {
        PositionCategory.bearing(src, dst)
    }

    Distance distance(Position src, Command dst) {
        distance(src, dst.target)
    }

    Distance distance(Map src, Command dst) {
        distance(dst.target, src)
    }

    Distance distance(DroneWrapper src, Command dst) {
        distance(src.position, dst.target)
    }

    Distance distance(Command src, Command dst) {
        distance(src.target, dst.target)
    }

    Distance distance(Command src, DroneWrapper dst) {
        distance(src.target, dst.position)
    }

    Distance distance(Command src, Position dst) {
        distance(src.target, dst)
    }

    Distance distance(Command src, Map dst) {
        distance(src.target, dst)
    }

    Distance distance(Command src, double lat, double lon) {
        distance(src.target, lat, lon)
    }

    Position target(Command src, Number distance, double bearing) {
        target(src.target, distance, bearing)
    }

    Position target(Command src, Distance distance, double bearing) {
        target(src.target, distance, bearing)
    }

    double bearing(Position src, Command dst) {
        bearing(src, dst.target)
    }

    double bearing(DroneWrapper src, Command dst) {
        bearing(src.position, dst.target)
    }

    double bearing(Command src, DroneWrapper dst) {
        bearing(src.target, dst.position)
    }

    double bearing(Command src, Command dst) {
        bearing(src.target, dst.target)
    }

    double bearing(Command src, Position dst) {
        bearing(src.target, dst)
    }

    double bearing(Command src, Map dst) {
        bearing(src.target, dst)
    }

    double bearing(Command src, double lat, double lon) {
        bearing(src.target, lat, lon)
    }

    double bearing(Map src, DroneWrapper dst) {
        CoordUtils.calcBearing(src, dst.position)
    }

    double bearing(Map src, Command dst) {
        CoordUtils.calcBearing(src, dst.target)
    }
}
