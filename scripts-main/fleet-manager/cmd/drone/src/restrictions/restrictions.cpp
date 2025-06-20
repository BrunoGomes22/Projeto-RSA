#include "drone_core/restrictions/restrictions.hpp"
#include "drone_core/restrictions/geofence.hpp"

RestrictionsChecker::RestrictionsChecker(rclcpp::Node::SharedPtr node_,
		std::shared_ptr<Telemetry> telem,
		std::shared_ptr<StatusTracker> tracker,
		const RestrictionsConfigs& configs) {
	this->node_ = node_;
	this->telem = telem;
	this->tracker = tracker;
	this->configs = configs;

	this->geofence = GeoFence();

	std::cout << "[Restrictions Checker] Geofence center: " << this->geofence.center->y << ", " << this->geofence.center->x << std::endl;
}

bool RestrictionsChecker::check(std::string cmd, Json::Value root) {
	this->cmd = cmd;
	this->root = root;

	this->lat = root["lat"].asFloat();
	this->lon = root["lon"].asFloat();
	this->alt = root["height"].asFloat();

	if (cmd == "takeoff") {
		return
			on_ground() &&
			valid_altitude()
		;
	}
	else if (cmd == "return" || cmd == "land") {
		return
			valid_state()
		;
	}
	else if (cmd == "goto" || cmd == "move") {
		return
			valid_state() &&
			null_island() &&
			valid_altitude()
			//within_geofence()
		;
	}
	else {
		return true;
	}
}

bool RestrictionsChecker::valid_state(std::string cmd) {
	if (!cmd.empty()) this->cmd = cmd;

	// Can only be executed if taking off or in air
	if (
		!(this->telem->landed_state() == Telemetry::LandedState::InAir || this->telem->landed_state() == Telemetry::LandedState::TakingOff) &&
		!(this->telem->flight_mode() == Telemetry::FlightMode::Takeoff || this->telem->flight_mode() == Telemetry::FlightMode::Hold)
	) {
		RCLCPP_ERROR(this->node_->get_logger(), ("Command \"" + this->cmd + "\" failed: drone must be taking off or hold.").c_str());
		this->tracker->track_cmd_failure(this->cmd, "drone must be taking off or hold");
		return false;
	}
	return true;
}

bool RestrictionsChecker::on_ground(std::string cmd) {
	if (!cmd.empty()) this->cmd = cmd;

	// print landed state and flight mode
	if (!(
		this->telem->landed_state() == Telemetry::LandedState::OnGround ||
		this->telem->flight_mode() == Telemetry::FlightMode::Ready
	)) {
		RCLCPP_ERROR(this->node_->get_logger(), ("Command \"" + this->cmd + "\" failed: drone must be on the ground.").c_str());
		this->tracker->track_cmd_failure(this->cmd, "drone must be on the ground");
		return false;
	}
	return true;
}

// Reference to Null Island (0, 0)
bool RestrictionsChecker::null_island(std::string cmd, double lat, double lon) {
	if (!cmd.empty()) this->cmd = cmd;
	if (!std::isnan(lat)) this->lat = lat;
	if (!std::isnan(lon)) this->lon = lon;

	if (this->lat == 0 && this->lon == 0) {
		RCLCPP_ERROR(this->node_->get_logger(), ("Command \"" + this->cmd + "\" failed: cannot go to Null Island.").c_str());
		this->tracker->track_cmd_failure(this->cmd, "cannot go to Null Island");
		return false;
	}
	return true;
}

bool RestrictionsChecker::valid_altitude(double alt) {
	if (!std::isnan(alt)) this->alt = alt;

	if (this->alt > configs.max_alt) {
		RCLCPP_ERROR(this->node_->get_logger(), ("Command \"" + this->cmd + "\" failed: altitude must be between 0 and " + std::to_string(configs.max_alt) + " meters.").c_str());
		this->tracker->track_cmd_failure(this->cmd, "altitude must be between 0 and " + std::to_string(configs.max_alt) + " meters");
		return false;
	}
	return true;
}

bool RestrictionsChecker::within_geofence(std::string cmd, double lat, double lon) {
	if (!cmd.empty()) this->cmd = cmd;
	if (!std::isnan(lat)) this->lat = lat;
	if (!std::isnan(lon)) this->lon = lon;

	if (!this->geofence.inside_fence(std::to_string(this->lat), std::to_string(this->lon))) {
		RCLCPP_ERROR(this->node_->get_logger(), ("Command \"" + this->cmd + "\" failed: cannot go outside the Geofence.").c_str());
		this->tracker->track_cmd_failure(this->cmd, "cannot go outside the Geofence");
		return false;
	}
	return true;
}

