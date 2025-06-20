#include "drone_core/coord_utils.hpp"

#include <math.h>

namespace coord_utils {
	static long double r_earth = 6371000;

	float toPositiveAngle(float angle) {
		angle = fmod(angle, 360);
		while (angle < 0) {
			angle += 360.0;
		}

		return angle;
	}

	long double toDegrees(const long double n) {
		return n * (180 / M_PI);
	}

	long double toRadians(const long double n) {
		return n * (M_PI / 180);
	}

	// The following method was based on https://www.geeksforgeeks.org/program-distance-two-points-earth/
	long double distance(long double lat1, long double long1, long double lat2, long double long2) {
		// Convert the latitudes and longitudes from degree to radians.
		lat1 = toRadians(lat1);
		long1 = toRadians(long1);
		lat2 = toRadians(lat2);
		long2 = toRadians(long2);

		// Haversine Formula
		long double dlong = long2 - long1;
		long double dlat = lat2 - lat1;

		long double ans = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong / 2), 2);

		ans = 2 * asin(sqrt(ans));

		// Calculate the result
		return ans * r_earth;
	}

	std::tuple<double, double> rotate(float heading, double x, double y) {
		heading *= -1;
		return std::make_tuple(x * cos(heading) - y * sin(heading), y * cos(heading) + x * sin(heading));
	}

	std::tuple<double, double, double> calcTranslatedPosition(float heading, double north, double east, double forward, double right) {
		std::tuple<double, double> dx_dy = rotate(toRadians(heading), right, forward);
		double new_north = north + std::get<1>(dx_dy);
		double new_east = east + std::get<0>(dx_dy);
		double yaw = toDegrees(atan2((new_east-east), (new_north-north)));
		return std::make_tuple(new_north, new_east, yaw);
	}

	std::tuple<double, double> calcTranslatedCoords(float heading, double lat, double lon, double x, double y) {
		std::tuple<double, double> dx_dy = rotate(toRadians(heading), x, y);
		double new_lat = lat + toDegrees(std::get<1>(dx_dy) / r_earth);
		double new_lon = lon + toDegrees(std::get<0>(dx_dy) / r_earth) / cos(toRadians(lat));
		return std::make_tuple(new_lat, new_lon);
	}

	float calcYaw(double currLat, double currLon, double newLat, double newLon) {
		double lonDiff = toRadians(newLon - currLon);
		currLat = toRadians(currLat);
		currLon = toRadians(currLon);
		newLat = toRadians(newLat);
		newLon = toRadians(newLon);
		double x = cos(newLat) * sin(lonDiff);
		double y = cos(currLat) * sin(newLat) - sin(currLat) * cos(newLat) * cos(lonDiff);
		return toDegrees(atan2(x,y));
	}
}