#include "drone_core/cmd_handler.hpp"

#include <unistd.h>

#include "drone_core/drone_utils.hpp"
#include "drone_core/coord_utils.hpp"

CommandHandler::CommandHandler(rclcpp::Node::SharedPtr node_, bool move_as_goto, std::shared_ptr<Telemetry> telem,
		std::shared_ptr<Action> action,	std::shared_ptr<Offboard> offboard,	std::shared_ptr<StatusTracker> tracker) {
	this->node_ = node_;
	this->telem = telem;
	this->action = action;
	this->offboard = offboard;
	this->tracker = tracker;
	this->last_target_alt = node_->get_parameter("takeoffAltitude").as_double();
	this->last_target_height = this->last_target_alt;
	this->move_as_goto = move_as_goto;

	float max_alt = this->max_alt();

	RestrictionsConfigs checkerConfigs;
	checkerConfigs.max_alt = max_alt;

	RCLCPP_INFO(node_->get_logger(), ("[Command Handler] Max altitude: " + std::to_string(max_alt)).c_str());

	this->checker = std::make_shared<RestrictionsChecker>(node_, telem, tracker, checkerConfigs);

	// print geofence center
	RCLCPP_INFO(node_->get_logger(), ("[Command Handler] Geofence center: " + std::to_string(checker->geofence.center->y) + ", " + std::to_string(checker->geofence.center->x)).c_str());
}

float CommandHandler::takeoff_alt() {
    return node_->get_parameter("takeoffAltitude").as_double();
}

float CommandHandler::return_alt() {
    return node_->get_parameter("returnAltitude").as_double();
}

float CommandHandler::max_speed() {
    return node_->get_parameter("maxSpeed").as_double();
}

float CommandHandler::max_alt() {
	return node_->get_parameter("maxAltitude").as_double();
}

void CommandHandler::handle_action_cmd_error(Action::Result action_result, std::string cmd) {
	std::ostringstream result_stream;
	result_stream << action_result;
	RCLCPP_ERROR(node_->get_logger(),
(				 "Action command \"" + cmd + "\" failed: " + result_stream.str()).c_str());
	tracker->track_cmd_failure(cmd, result_stream.str());
}

void CommandHandler::handle_offboard_cmd_error(Offboard::Result offb_result, std::string cmd) {
	std::ostringstream result_stream;
	result_stream << offb_result;
	RCLCPP_ERROR(node_->get_logger(),
(				 "Offboard command \"" + cmd + "\" failed: " + result_stream.str()).c_str());
	tracker->track_cmd_failure(cmd, result_stream.str());
}

bool CommandHandler::can_move(std::string cmd, Json::Value root) {
	RCLCPP_INFO(node_->get_logger(), "Altitude: %f", root["alt"].asFloat());
	RCLCPP_INFO(node_->get_logger(), "Height: %f", root["height"].asFloat());
	return this->checker->check(cmd, root);
}

void CommandHandler::arm() {
	Action::Result result = action->arm();
	if (result != Action::Result::Success)
		handle_action_cmd_error(result, "arm");
	else
		tracker->track_arm_disarm(false);
}

void CommandHandler::disarm() {
	Action::Result result = action->disarm();
	if (result != Action::Result::Success)
		handle_action_cmd_error(result, "disarm");
	else
		tracker->track_arm_disarm(true);
}

void CommandHandler::takeoff(Json::Value root) {
	if (root.isMember("alt")) {
		RCLCPP_INFO(node_->get_logger(), "Takeoff altitude provided");
		last_target_alt = root["alt"].asFloat() + telem->position().absolute_altitude_m;
		last_target_height = root["alt"].asFloat() + telem->position().relative_altitude_m;
	} else {
		RCLCPP_INFO(node_->get_logger(), "Takeoff altitude not provided, using default");
		root["alt"] = takeoff_alt();
		last_target_alt = takeoff_alt() + telem->position().absolute_altitude_m;
		last_target_height = takeoff_alt() + telem->position().relative_altitude_m;
	}
	
	action->set_takeoff_altitude(root["alt"].asFloat());

	root["height"] = root["alt"].asFloat();
	root["alt"] = last_target_alt;

	RCLCPP_INFO(node_->get_logger(), "Takeoff altitude: %f", root["height"].asFloat());

	if (!can_move("takeoff", root))
		return;

	Action::Result result = action->takeoff();
	if (result != Action::Result::Success)
		handle_action_cmd_error(result, "takeoff");
	else
		tracker->track_action(ActionEnum::TAKEOFF);
}

void CommandHandler::land() {
	Action::Result result = action->land();
	if (result != Action::Result::Success)
		handle_action_cmd_error(result, "land");
	else
		tracker->track_action(ActionEnum::LAND);
}

void CommandHandler::go_to(Json::Value root, bool is_move) {
	if (telem->flight_mode() == Telemetry::FlightMode::Offboard)
		stop_offb(true);

	// If no yaw angle is provided, default to current heading
	if (!root.isMember("yaw"))
		root["yaw"] = telem->attitude_euler().yaw_deg;

	// If yaw angle is null, face the location
	if (root["yaw"] == Json::nullValue)
		root["yaw"] = calcYaw(telem->position().latitude_deg, telem->position().longitude_deg,
							  root["lat"].asDouble(), root["lon"].asDouble());

	// If no altitude is provided, default to current altitude
	if (!root.isMember("alt") || root["alt"] == Json::nullValue) {
		if (telem->flight_mode() == Telemetry::FlightMode::Takeoff) {
			root["alt"] = last_target_alt;
			root["height"] = last_target_height;
		}
		else {
			root["alt"] = telem->position().absolute_altitude_m;
			root["height"] = telem->position().relative_altitude_m;
		}
	}

	// Set speed
	if (!root.isMember("speed") || root["speed"] == Json::nullValue)
		action->set_maximum_speed(max_speed());
	else
		action->set_maximum_speed(root["speed"].asFloat());

	if (!can_move("goto", root))
		return;

	Action::Result result = action->goto_location(
		root["lat"].asDouble(), root["lon"].asDouble(),
		root["alt"].asDouble(),
		root["yaw"].asFloat()
	);

    if (is_move) {
        if (result != Action::Result::Success)
            handle_action_cmd_error(result, "move");
    } else {
        if (result != Action::Result::Success)
            handle_action_cmd_error(result, "goto");
        else
			// TODO: maybe pass 'height' instead of 'alt'?
            tracker->track_goto(root["lat"].asDouble(), root["lon"].asDouble(), root["alt"].asFloat());
    }
}

void CommandHandler::return_to_launch(Json::Value root) {
	if (root.isMember("alt")) {
		action->set_return_to_launch_altitude(root["alt"].asFloat());
	} else {
		action->set_return_to_launch_altitude(return_alt());
	}

	if (!root.isMember("speed") || root["speed"] == Json::nullValue)
		action->set_maximum_speed(max_speed());
	else
		action->set_maximum_speed(root["speed"].asFloat());

	Action::Result result = action->return_to_launch();
	if (result != Action::Result::Success)
		handle_action_cmd_error(result, "return");
	else
		tracker->track_action(ActionEnum::RETURN);
}

void CommandHandler::go_to_geofence(Json::Value root) {
	root["lat"] = this->checker->geofence.center->x;
	root["lon"] = this->checker->geofence.center->y;

	if (!can_move("geofence", root))
		return;


	// set yaw to face the geofence center
	root["yaw"] = calcYaw(telem->position().latitude_deg, telem->position().longitude_deg,
						  root["lat"].asDouble(), root["lon"].asDouble());

	go_to(root, false);
	tracker->track_action(ActionEnum::GEOFENCE);
}

void CommandHandler::start_offb(bool silenced) {
	if (telem->flight_mode() == Telemetry::FlightMode::Offboard) {
		RCLCPP_ERROR(node_->get_logger(),
(					 std::string("Offboard command \"start\" failed: already started.")).c_str());
		tracker->track_cmd_failure("start_offboard", "already started");
		return;
	}
	// An initial setpoint is needed to start offboard mode
	offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
	Offboard::Result result = offboard->start();
	if (result != Offboard::Result::Success)
		handle_offboard_cmd_error(result, "start_offboard");
	else
		tracker->track_start_stop_offb(true, silenced);
}

void CommandHandler::stop_offb(bool silenced) {
	if (telem->flight_mode() != Telemetry::FlightMode::Offboard) {
		RCLCPP_ERROR(node_->get_logger(),
(					 std::string("Offboard command \"stop\" failed: offboard mode is not active.")).c_str());
		tracker->track_cmd_failure("stop_offboard", "offboard mode is not active");
		return;
	}
	Offboard::Result result = offboard->stop();
	if (result != Offboard::Result::Success)
		handle_offboard_cmd_error(result, "stop_offboard");
	else
		tracker->track_start_stop_offb(false, silenced);
}

void CommandHandler::position_ned(Json::Value root, bool is_move) {
	if(telem->flight_mode() != Telemetry::FlightMode::Offboard)
		start_offb(true);
	Offboard::PositionNedYaw position_ned{};
	if (root.isMember("north"))
		position_ned.north_m = root["north"].asFloat();
	if (root.isMember("east"))
		position_ned.east_m = root["east"].asFloat();
	if (root.isMember("down"))
		position_ned.down_m = root["down"].asFloat();
	if (root.isMember("yaw"))
		position_ned.yaw_deg = root["yaw"].asFloat();
	offboard->set_position_ned(position_ned);
	if (!is_move)
	    tracker->track_offboard(OffboardEnum::POSITION_NED, root["north"].asFloat(), root["east"].asFloat(), root["down"].asFloat());
}

void CommandHandler::velocity_ned(Json::Value root, bool is_turn) {
	if(telem->flight_mode() != Telemetry::FlightMode::Offboard)
		start_offb(true);
	Offboard::VelocityNedYaw velocity_ned{};
	if (root.isMember("north"))
		velocity_ned.north_m_s = root["north"].asFloat();
	if (root.isMember("east"))
		velocity_ned.east_m_s = root["east"].asFloat();
	if (root.isMember("down"))
		velocity_ned.down_m_s = root["down"].asFloat();
	if (root.isMember("yaw"))
		velocity_ned.yaw_deg = root["yaw"].asFloat();
	offboard->set_velocity_ned(velocity_ned);
	if (!is_turn)
	    tracker->track_offboard(OffboardEnum::VELOCITY_NED);
}

void CommandHandler::velocity_body(Json::Value root) {
	if(telem->flight_mode() != Telemetry::FlightMode::Offboard)
		start_offb(true);
	Offboard::VelocityBodyYawspeed velocity_body{};
	if (root.isMember("forward"))
		velocity_body.forward_m_s = root["forward"].asFloat();
	if (root.isMember("right"))
		velocity_body.right_m_s = root["right"].asFloat();
	if (root.isMember("down"))
		velocity_body.down_m_s = root["down"].asFloat();
	if (root.isMember("yaw"))
		velocity_body.yawspeed_deg_s = root["yaw"].asFloat();
	offboard->set_velocity_body(velocity_body);
	tracker->track_offboard(OffboardEnum::VELOCITY_BODY);
}

void CommandHandler::attitude(Json::Value root) {
	if(telem->flight_mode() != Telemetry::FlightMode::Offboard)
		start_offb(true);
	Offboard::Attitude attitude{};
	if (root.isMember("roll"))
		attitude.roll_deg = root["roll"].asFloat();
	if (root.isMember("pitch"))
		attitude.pitch_deg = root["pitch"].asFloat();
	if (root.isMember("yaw"))
		attitude.yaw_deg = root["yaw"].asFloat();
	if (root.isMember("thrust"))
		attitude.thrust_value = root["thrust"].asFloat();
	offboard->set_attitude(attitude);
	tracker->track_offboard(OffboardEnum::ATTITUDE);
}

void CommandHandler::attitude_rate(Json::Value root) {
	if(telem->flight_mode() != Telemetry::FlightMode::Offboard)
		start_offb(true);
	Offboard::AttitudeRate attitude{};
	if (root.isMember("roll"))
		attitude.roll_deg_s = root["roll"].asFloat();
	if (root.isMember("pitch"))
		attitude.pitch_deg_s = root["pitch"].asFloat();
	if (root.isMember("yaw"))
		attitude.yaw_deg_s = root["yaw"].asFloat();
	if (root.isMember("thrust"))
		attitude.thrust_value = root["thrust"].asFloat();
	offboard->set_attitude_rate(attitude);
	tracker->track_offboard(OffboardEnum::ATTITUDE_RATE);
}

void CommandHandler::actuator_control(Json::Value root) {
	if(telem->flight_mode() != Telemetry::FlightMode::Offboard)
		start_offb(true);
	if (!(root.isMember("group0") && root.isMember("group1"))) {
		RCLCPP_ERROR(node_->get_logger(),
(					 "Offboard command \"" + root["cmd"].asString() + "\" failed: missing parameters").c_str());
		return;
	}

	// Retrieve each group from json
	Json::Value json_grp0 = root["group0"];
	Json::Value json_grp1 = root["group1"];

	// Check if group size is equal to 8
	if (json_grp0.size() != 8 || json_grp1.size() != 8) {
		RCLCPP_ERROR(node_->get_logger(),
(					 "Offboard command \"" + root["cmd"].asString() + " failed: each group must have 8 values").c_str());
		return;
	}

	// Initialize each actuator control group
	Offboard::ActuatorControlGroup group0, group1;
	group0.controls = {
			json_grp0[0].asFloat(), json_grp0[1].asFloat(), json_grp0[2].asFloat(), json_grp0[3].asFloat(),
			json_grp0[4].asFloat(), json_grp0[5].asFloat(), json_grp0[6].asFloat(), json_grp0[7].asFloat()
	};
	group1.controls = {
			json_grp1[0].asFloat(), json_grp1[1].asFloat(), json_grp1[2].asFloat(), json_grp1[3].asFloat(),
			json_grp1[4].asFloat(), json_grp1[5].asFloat(), json_grp1[6].asFloat(), json_grp1[7].asFloat()
	};

	// Initialize actuator control
	Offboard::ActuatorControl act_ctrl;
	act_ctrl.groups = {group0, group1};
	offboard->set_actuator_control(act_ctrl);
	tracker->track_offboard(OffboardEnum::ACTUATOR_CONTROL);
}

void CommandHandler::turn(Json::Value root) {
	if (!can_move("velocity_ned", root))
		return;
	Json::Value turn;
	float angle = root["deg"].asFloat() + telem->attitude_euler().yaw_deg;
	turn["yaw"] = angle;
	velocity_ned(turn, true);
	tracker->track_turn(angle, root["deg"].asFloat());
}

void CommandHandler::move(Json::Value root) {
	Json::Value coords;
    // Calculate latitude and longitude; this is calculated even if using position_ned as the actual command since it is
    // useful to publish an approximate target coordinate
    std::tuple<double, double> lat_lon =
            calcTranslatedCoords(telem->attitude_euler().yaw_deg,
                                 telem->position().latitude_deg,
                                 telem->position().longitude_deg,
                                 root["x"].asDouble(), root["y"].asDouble());
    coords["lat"] = std::get<0>(lat_lon);
    coords["lon"] = std::get<1>(lat_lon);

    // Calculate altitude
    coords["alt"] = telem->position().absolute_altitude_m + root["z"].asFloat();
	coords["height"] = telem->position().relative_altitude_m + root["z"].asFloat();

	if (!can_move("move", coords))
		return;

	if (move_as_goto) {
        coords["yaw"] = root["yaw"];
        coords["speed"] = root["speed"];

        // Send command
        go_to(coords, true);
    } else {
    	// Calculate new position
    	std::tuple<double, double, double> new_pos =
    			calcTranslatedPosition(telem->attitude_euler().yaw_deg,
    								 telem->position_velocity_ned().position.north_m,
    								 telem->position_velocity_ned().position.east_m,
    								 root["y"].asDouble(), root["x"].asDouble());
    	coords["north"] = std::get<0>(new_pos);
    	coords["east"] = std::get<1>(new_pos);

    	// Calculate altitude
        coords["down"] = telem->position_velocity_ned().position.down_m - root["z"].asFloat();

        // If no yaw angle is provided or it is null, face the next waypoint
        if (!root.isMember("yaw") || root["yaw"] == Json::nullValue)
            coords["yaw"] = std::get<2>(new_pos);

        // Set speed
        if (!root.isMember("speed") || root["speed"] == Json::nullValue)
            action->set_maximum_speed(max_speed());
        else
            action->set_maximum_speed(root["speed"].asFloat());

    	// Send command
    	position_ned(coords, true);
    }

    tracker->track_move(root["y"].asDouble(), root["x"].asDouble(), root["z"].asDouble(), coords, move_as_goto);
}

void CommandHandler::cancel() {
	if(telem->flight_mode() != Telemetry::FlightMode::Offboard)
		start_offb(true);
	stop_offb(true);
	tracker->publish_custom_status(CustomCmdEnum::CANCEL, "success");
}