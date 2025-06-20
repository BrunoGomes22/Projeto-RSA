#ifndef FLEETMAN_RESTRICTIONS_HPP
#define FLEETMAN_RESTRICTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "drone_core/status_tracker.hpp"
#include "drone_core/restrictions/geofence.hpp"

struct RestrictionsConfigs {
	float max_alt;
};

class RestrictionsChecker {
public:
	RestrictionsChecker(rclcpp::Node::SharedPtr node_,
				  std::shared_ptr<Telemetry> telem,
				  std::shared_ptr<StatusTracker> tracker,
				  const RestrictionsConfigs& configs);

	bool check(std::string cmd, Json::Value root);

	bool valid_state(std::string cmd = "");
	bool on_ground(std::string cmd = "");
	bool null_island(std::string cmd = "", double lat = std::numeric_limits<double>::quiet_NaN(), double lon = std::numeric_limits<double>::quiet_NaN());
	bool valid_altitude(double alt = std::numeric_limits<double>::quiet_NaN());
	bool within_geofence(std::string cmd = "", double lat = std::numeric_limits<double>::quiet_NaN(), double lon= std::numeric_limits<double>::quiet_NaN());

	GeoFence geofence;


private:
	rclcpp::Node::SharedPtr node_;
	std::shared_ptr<Telemetry> telem;
	std::shared_ptr<StatusTracker> tracker;
	RestrictionsConfigs configs;

	std::string cmd;
	Json::Value root;
	double lat;
	double lon;
	double alt;
};

#endif //FLEETMAN_RESTRICTIONS_HPP