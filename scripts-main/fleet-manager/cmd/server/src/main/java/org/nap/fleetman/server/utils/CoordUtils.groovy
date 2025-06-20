package org.nap.fleetman.server.utils

import org.nap.fleetman.server.model.dsl.Distance
import org.nap.fleetman.server.model.dsl.DistanceUnit

class CoordUtils {
    // Earth radius in meters
    private static final int earthRadius = 6371000

    static Distance calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        // Convert the latitudes and longitudes from degree to radians.
        lat1 = Math.toRadians(lat1)
        lon1 = Math.toRadians(lon1)
        lat2 = Math.toRadians(lat2)
        lon2 = Math.toRadians(lon2)

        // Haversine Formula
        def dlon = lon2 - lon1
        def dlat = lat2 - lat1

        def distance = (Math.sin(dlat / 2)**2) + Math.cos(lat1) * Math.cos(lat2) * (Math.sin(dlon / 2)**2)
        distance = 2 * Math.asin(Math.sqrt(distance)) * earthRadius

        // Return distance
        new Distance(distance, DistanceUnit.meter)
    }

    static def calcTarget(double lat, double lon, Number dist, double bearing) {
        bearing = Math.toRadians(bearing)
        def latRadians = Math.toRadians(lat)
        def lonRadians = Math.toRadians(lon)

        def lat2 = Math.asin(Math.sin(latRadians) * Math.cos(dist / earthRadius) + Math.cos(latRadians) * Math.sin(dist / earthRadius) * Math.cos(bearing))
        def lon2 = lonRadians + Math.atan2(Math.sin(bearing) * Math.sin(dist / earthRadius) * Math.cos(latRadians), Math.cos(dist / earthRadius) - Math.sin(latRadians) * Math.sin(lat2))

        [lat: Math.toDegrees(lat2), lon: Math.toDegrees(lon2)]
    }

    static double calcBearing(double lat1, double lon1, double lat2, double lon2) {
        double lonDiff = Math.toRadians(lon2 - lon1)
        def currLat = Math.toRadians(lat1)
        def newLat = Math.toRadians(lat2)
        double x = Math.cos(newLat) * Math.sin(lonDiff)
        double y = Math.cos(currLat) * Math.sin(newLat) - Math.sin(currLat) * Math.cos(newLat) * Math.cos(lonDiff)
        Math.toDegrees(Math.atan2(x, y))
    }
}
