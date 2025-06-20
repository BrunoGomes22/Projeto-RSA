package org.nap.fleetman.server.mission.dsl

import org.nap.fleetman.server.model.drone.Command
import org.nap.fleetman.server.model.dsl.Distance
import org.nap.fleetman.server.model.dsl.DistanceUnit
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.model.telemetry.Position
import org.nap.fleetman.server.utils.CoordUtils

class PositionCategory {
    static Distance distance(Position src, Position dst) {
        double distance = (double) CoordUtils.calculateDistance(src.lat, src.lon, dst.lat, dst.lon).amount
        new Distance(Math.sqrt(distance**2 + (dst.alt - src.alt)**2), DistanceUnit.meter)
    }

    static Distance distance(Position src, double lat, double lon) {
        CoordUtils.calculateDistance(src.lat, src.lon, lat, lon)
    }

    static Distance distance(Position src, Map dst) {
        if (dst.containsKey('alt'))
            distance(src, new Position(dst.lat, dst.lon, dst.alt))
        else
            CoordUtils.calculateDistance(src.lat, src.lon, dst.lat, dst.lon)
    }

    static Distance distance(Position src, DroneWrapper dst) {
        distance(src, dst.position)
    }

    static Distance distance(Position src, Command dst) {
        distance(src, dst.target)
    }

    static Distance distance(Map src, Position dst) {
        distance(dst, src)
    }

    static Distance distance(Map src, Command dst) {
        distance(dst.target, src)
    }

    static Distance distance(Map src, DroneWrapper dst) {
        distance(dst.position, src)
    }

    static Distance distance(Map src, double lat, double lon) {
        CoordUtils.calculateDistance(src.lat, src.lon, lat, lon)
    }

    static Distance distance(Map src, Map dst) {
        if (src.containsKey('alt') && dst.containsKey('alt'))
            distance(new Position(src.lat, src.lon, src.alt), new Position(dst.lat, dst.lon, dst.alt))
        else
            CoordUtils.calculateDistance(src.lat, src.lon, dst.lat, dst.lon)
    }

    static Distance distance(DroneWrapper src, DroneWrapper dst) {
        distance(src.position, dst.position)
    }

    static Distance distance(DroneWrapper src, Position dst) {
        distance(src.position, dst)
    }

    static Distance distance(DroneWrapper src, Map dst) {
        distance(src.position, dst)
    }

    static Distance distance(DroneWrapper src, double lat, double lon) {
        distance(src.position, lat, lon)
    }

    static Distance distance(DroneWrapper src, Command dst) {
        distance(src.position, dst.target)
    }

    static Distance distance(Command src, Command dst) {
        distance(src.target, dst.target)
    }

    static Distance distance(Command src, DroneWrapper dst) {
        distance(src.target, dst.position)
    }

    static Distance distance(Command src, Position dst) {
        distance(src.target, dst)
    }

    static Distance distance(Command src, Map dst) {
        distance(src.target, dst)
    }

    static Distance distance(Command src, double lat, double lon) {
        distance(src.target, lat, lon)
    }

    static Position target(Position src, Number distance, double bearing) {
        def coords = CoordUtils.calcTarget(src.lat, src.lon, distance, bearing)
        new Position(coords.lat, coords.lon, src.alt)
    }

    static Position target(Position src, Distance distance, double bearing) {
        target(src, distance.convertTo(DistanceUnit.meter), bearing)
    }

    static Position target(DroneWrapper src, Number distance, double bearing) {
        target(src.position, distance, bearing)
    }

    static Position target(DroneWrapper src, Distance distance, double bearing) {
        target(src.position, distance, bearing)
    }

    static Position target(Command src, Number distance, double bearing) {
        target(src.target, distance, bearing)
    }

    static Position target(Command src, Distance distance, double bearing) {
        target(src.target, distance, bearing)
    }

    static def target(Map src, Number distance, double bearing) {
        if (src.containsKey('alt'))
            target(new Position(src.lat, src.lon, src.alt as Float), distance, bearing)
        else
            CoordUtils.calcTarget(src.lat, src.lon, distance, bearing)
    }

    static def target(Map src, Distance distance, double bearing) {
        target(src, distance.convertTo(DistanceUnit.meter), bearing)
    }

    static double bearing(Position src, double lat, double lon) {
        CoordUtils.calcBearing(src.lat, src.lon, lat, lon)
    }

    static double bearing(Position src, Position dst) {
        CoordUtils.calcBearing(src.lat, src.lon, dst.lat, dst.lon)
    }

    static double bearing(Position src, DroneWrapper dst) {
        bearing(src, dst.position)
    }

    static double bearing(Position src, Command dst) {
        bearing(src, dst.target)
    }

    static double bearing(Position src, Map dst) {
        CoordUtils.calcBearing(src.lat, src.lon, dst.lat, dst.lon)
    }

    static double bearing(DroneWrapper src, DroneWrapper dst) {
        bearing(src.position, dst.position)
    }

    static double bearing(DroneWrapper src, Position dst) {
        bearing(src.position, dst)
    }

    static double bearing(DroneWrapper src, Map dst) {
        bearing(src.position, dst)
    }

    static double bearing(DroneWrapper src, double lat, double lon) {
        bearing(src.position, lat, lon)
    }

    static double bearing(DroneWrapper src, Command dst) {
        bearing(src.position, dst.target)
    }

    static double bearing(Command src, DroneWrapper dst) {
        bearing(src.target, dst.position)
    }

    static double bearing(Command src, Command dst) {
        bearing(src.target, dst.target)
    }

    static double bearing(Command src, Position dst) {
        bearing(src.target, dst)
    }

    static double bearing(Command src, Map dst) {
        bearing(src.target, dst)
    }

    static double bearing(Command src, double lat, double lon) {
        bearing(src.target, lat, lon)
    }

    static double bearing(Map src, double lat, double lon) {
        CoordUtils.calcBearing(src.lat, src.lon, lat, lon)
    }

    static double bearing(Map src, Position dst) {
        CoordUtils.calcBearing(src.lat, src.lon, dst.lat, dst.lon)
    }

    static double bearing(Map src, DroneWrapper dst) {
        CoordUtils.calcBearing(src, dst.position)
    }

    static double bearing(Map src, Command dst) {
        CoordUtils.calcBearing(src, dst.target)
    }

    static def forward(Map position, Distance dist) {
        def pos = target(position, dist, position.bearing)
        [lat: pos.lat, lon: pos.lon, alt: pos.alt, bearing: position.bearing]
    }

    static def backward(Map position, Distance dist) {
        def pos = target(position, dist, position.bearing + 180)
        [lat: pos.lat, lon: pos.lon, alt: pos.alt, bearing: position.bearing -180]
    }

    static def left(Map position, Distance dist) {
        def pos = target(position, dist, position.bearing - 90)
        [lat: pos.lat, lon: pos.lon, alt: pos.alt, bearing: position.bearing - 90]
    }

    static def right(Map position, Distance dist) {
        def pos = target(position, dist, position.bearing + 90)
        [lat: pos.lat, lon: pos.lon, alt: pos.alt, bearing: position.bearing -90]
    }
}
