#ifndef FLEETMAN_DRONE_COORD_UTILS_H
#define FLEETMAN_DRONE_COORD_UTILS_H

#include<tuple>

namespace coord_utils {
	float toPositiveAngle(float angle);
	long double distance(long double lat1, long double long1, long double lat2, long double long2);
	std::tuple<double, double, double> calcTranslatedPosition(float heading, double north, double east, double forward, double right);
	std::tuple<double, double> calcTranslatedCoords(float heading, double lat, double lon, double x, double y);
	float calcYaw(double currLat, double currLon, double newLat, double newLon);
}

#endif //FLEETMAN_DRONE_COORD_UTILS_H
