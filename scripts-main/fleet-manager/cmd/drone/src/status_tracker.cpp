#include "drone_core/status_tracker.hpp"

#include <iostream>
#include <unistd.h>

// Mapping of command status to string
const std::unordered_map<StatusTracker::CommandStatus, std::string> StatusTracker::cmd_status_table = {
		{CommandStatus::START, "start"},
		{CommandStatus::STOP, "stop"},
		{CommandStatus::CANCEL, "cancel"},
		{CommandStatus::FINISH, "finish"},
		{CommandStatus::QUEUED, "queued"},
		{CommandStatus::SUCCESS, "success"},
		{CommandStatus::FAILURE, "failure"}
};

// Mapping of system status to string
const std::unordered_map<StatusTracker::SystemStatus, std::string> StatusTracker::sys_status_table = {
	{SystemStatus::CONNECT, "connect"},
	{SystemStatus::DISCONNECT, "disconnect"},
	{SystemStatus::DISARM, "disarm"},
	{SystemStatus::RETURN, "return"},
	{SystemStatus::STOP_OFFBOARD, "stop_offboard"},
	{SystemStatus::START_MANUAL, "start_manual"},
	{SystemStatus::STOP_MANUAL, "stop_manual"}
};

StatusTracker::StatusTracker(rclcpp::Node::SharedPtr node_, std::string drone_id, std::shared_ptr<Telemetry> telem, std::shared_ptr<Action> action) {
	this->drone_id = drone_id;
	this->node_ = node_;
	this->telem = telem;
	this->action = action;
	this->pub = node_->create_publisher<std_msgs::msg::String>(node_->get_parameter("statusTopic").value_to_string(), 64);

	// Configure json builder
	builder["indentation"] = "";

	// Initialize internal variables
	was_armed = telem->armed();
	manually_disarmed = false;
	tracking_cmd = false;
	manually_return = false;
    queued_goto = false;
	yaw = NAN;
    north = NAN;
    east = NAN;
    down = NAN;

	// Start the command completion tracking thread
	std::thread tracker(&StatusTracker::track, this);
	tracker.detach();
}

double StatusTracker::acceptance_radius() {
    return node_->get_parameter("acceptanceRadius").as_double();
}

// Convert command status enum to string
std::string StatusTracker::cmd_status_to_string(CommandStatus status) {
	if (auto it = cmd_status_table.find(status); it != cmd_status_table.end())
		return it->second;
	return nullptr;
}

// Convert system status enum to string
std::string StatusTracker::sys_status_to_string(SystemStatus status) {
	if (auto it = sys_status_table.find(status); it != sys_status_table.end())
		return it->second;
	return nullptr;
}

// Build json node with coordinates
Json::Value StatusTracker::get_coords_node(double lat_, double lon_, float alt_) {
	if (std::isnan(lat_)) {
		lat_ = this->lat;
		lon_ = this->lon;
		alt_ = this->alt;
	}
	Json::Value node;
	node["coords"] = Json::nullValue;
	node["coords"]["lat"] = lat_;
	node["coords"]["lon"] = lon_;
	node["coords"]["alt"] = alt_;
	return node;
}

Json::Value StatusTracker::get_move_coords_node() {
    Json::Value node = get_coords_node();
    node["pos"]["forward"] = forward;
    node["pos"]["right"] = right;
    node["pos"]["up"] = up;
    return node;
}

// Fill constant fields in json message
Json::Value StatusTracker::get_base_message() {
	Json::Value root;
	root["droneId"] = drone_id;
	root["timestamp"]  = get_timestamp();
	return root;
}

void StatusTracker::publish_custom_status(CustomCmdEnum cmd, std::string status) {
	Json::Value root = get_base_message();
	root["command"] = enum_to_string(cmd);
	root["state"] = status;
	publish_json(root, builder, pub);
}

// Publish a message with a command's status
void StatusTracker::publish_cmd_status(std::string cmd, CommandStatus status, Json::Value value) {
    Json::FastWriter fastWriter;
	Json::Value root = get_base_message();
	root["command"] = cmd;
	root["state"] = cmd_status_to_string(status);
	if (value != Json::nullValue) {
		for (auto key : value.getMemberNames())
			root[key] = value[key];
	}
	publish_json(root, builder, pub);
}

void StatusTracker::publish_cmd_status(int cmd, CommandStatus status, Json::Value value) {
	publish_cmd_status(enum_to_string(cmd), status, value);
}

void StatusTracker::publish_cmd_status(ActionEnum cmd, CommandStatus status, Json::Value value) {
	publish_cmd_status(enum_to_string(cmd), status, value);
}

void StatusTracker::publish_cmd_status(OffboardEnum cmd, CommandStatus status, Json::Value value) {
	publish_cmd_status(enum_to_string(cmd), status, value);
}

// Publish message with system status
void StatusTracker::publish_sys_status(SystemStatus status) {
	Json::Value root = get_base_message();
	root["component"] = "system";
	root["state"] = sys_status_to_string(status);
	publish_json(root, builder, pub);
}

// Register a canceled command
void StatusTracker::cancel_cmd() {
	// Dequeue a go to command if it's queued
	if (queued_goto)
		dequeue_goto();
	if (!std::isnan(yaw))
		yaw = NAN;
	if (!std::isnan(north)) {
        north = NAN;
        east = NAN;
        down = NAN;
	}

	CommandStatus status = CommandStatus::CANCEL;
	// If the tracked command is a Offboard command, except position_ned, send a stop status message instead of cancel
	if (tracking_cmd > 9 && tracking_cmd < 15)
	    status = CommandStatus::STOP;

    // If the tracked command is a goto or a move, log their coordinates
	Json::Value node = Json::nullValue;
	if (tracking_cmd == 4) {
	    node = get_coords_node();
    } else if (tracking_cmd == 16) {
        node = get_move_coords_node();
    }
	publish_cmd_status(tracking_cmd, status, node);
}

// Cancel a queued go to command
void StatusTracker::dequeue_goto() {
	publish_cmd_status(ActionEnum::GOTO, CommandStatus::CANCEL, get_coords_node());
	queued_goto = false;
}

// Register a finished command
void StatusTracker::finish_cmd(Json::Value value) {
	publish_cmd_status(tracking_cmd, CommandStatus::FINISH, value);
	// Reset tracked command
	tracking_cmd = 0;
}

// Update mavlink system connection status
void StatusTracker::update_system_connection(bool is_connected) {
	publish_sys_status(is_connected ? SystemStatus::CONNECT : SystemStatus::DISCONNECT);
}

// Register command that failed
void StatusTracker::track_cmd_failure(std::string cmd, std::string error) {
	Json::Value node = Json::nullValue;
	node["error"] = error;
	publish_cmd_status(cmd, CommandStatus::FAILURE, node);
}

// Register start/stop offboard mode command
void StatusTracker::track_start_stop_offb(bool start, bool silenced) {
	// Commands that are being executed are canceled
	if (tracking_cmd) {
		cancel_cmd();
		// Reset tracked command
		tracking_cmd = 0;
	}
	// While current offboard status doesn't match offboard trigger
	while((telem->flight_mode() == Telemetry::FlightMode::Offboard) != start) {
		usleep(50000);
	}

	if (!silenced)
		publish_cmd_status(start ? OffboardEnum::START : OffboardEnum::STOP , CommandStatus::SUCCESS);
}

// Register a successful arm/disarm command
void StatusTracker::track_arm_disarm(bool is_disarm) {
	manually_disarmed = is_disarm;
	if (is_disarm)
		was_armed = false;

	while(telem->armed() == is_disarm) {
		usleep(50000);
	}
	publish_cmd_status(is_disarm, CommandStatus::SUCCESS);
}

// Start tracking action command (excluding go to)
void StatusTracker::track_action(ActionEnum cmd) {
	// Lock to reduce race condition chance
	std::unique_lock<std::mutex> lck(mutex);
	cond_var.wait(lck);

	// Cancel current command if the drone was already performing one
	if (tracking_cmd) {
		cancel_cmd();
	}
	// If the drone is in offboard mode and the new command isn't, register that offboard mode is stopped
	if (telem->flight_mode() == Telemetry::FlightMode::Offboard) {
		publish_sys_status(SystemStatus::STOP_OFFBOARD);
	}

	Json::Value node = Json::nullValue;
	switch(cmd) {
		case (ActionEnum::TAKEOFF):
			node["altitude"] = std::get<1>(action->get_takeoff_altitude());
			this->alt = std::get<1>(action->get_takeoff_altitude()) + telem->position().absolute_altitude_m;
			break;
		case(ActionEnum::RETURN):
			manually_return = true;
			node["altitude"] = std::get<1>(action->get_return_to_launch_altitude());
			break;
		default:
			break;
	}

	// Start the new command
	publish_cmd_status(cmd, CommandStatus::START, node);
	tracking_cmd = static_cast<int>(cmd);
}

// Start tracking go to command
void StatusTracker::track_goto(double lat, double lon, float alt) {
	// Lock to reduce race condition chance
	std::unique_lock<std::mutex> lck(mutex);
	cond_var.wait(lck);

	// If the drone is currently taking off
	if (tracking_cmd == 2) {
		// Cancel the current queued goto command if there's one
		if (queued_goto) {
			dequeue_goto();
		}
		// Queue this go to command
		publish_cmd_status(ActionEnum::GOTO, CommandStatus::QUEUED, get_coords_node(lat, lon, alt));
		queued_goto = true;
	} else {
		// If the drone is already executing a go to command, cancel it
		if (tracking_cmd == 4 || tracking_cmd == 16) {
			cancel_cmd();
		}
		// Start this go to command
		tracking_cmd = 4;
		publish_cmd_status(ActionEnum::GOTO, CommandStatus::START, get_coords_node(lat, lon, alt));
	}
	// Update target coordinates
	this->lat = lat;
	this->lon = lon;
	this->alt = alt;
}

// Start tracking offboard command
void StatusTracker::track_offboard(OffboardEnum cmd, float north, float east, float down, bool isMove) {
	// Lock to reduce race condition chance
	std::unique_lock<std::mutex> lck(mutex);
	cond_var.wait(lck);

	// Cancel current command if the drone was already performing one
	if (tracking_cmd) {
		cancel_cmd();
	}

	if (cmd == OffboardEnum::POSITION_NED) {
		this->north = north;
		this->east = east;
		this->down = down;
	}

	// Start the new command
	if (!isMove) {
        publish_cmd_status(cmd, CommandStatus::START);
        tracking_cmd = static_cast<int>(cmd);
	}
}

void StatusTracker::track_turn(float yaw, float relative_deg) {
	// Lock to reduce race condition chance
	std::unique_lock<std::mutex> lck(mutex);
	cond_var.wait(lck);

	// Cancel current command if the drone was already performing one
	if (tracking_cmd) {
		cancel_cmd();
	}

	Json::Value node;
	node["degrees"] = relative_deg;

	// Start turn
	publish_cmd_status("turn", CommandStatus::START, node);
    tracking_cmd = static_cast<int>(CustomCmdEnum::TURN);
	this->yaw = yaw;
}

void StatusTracker::track_move(double forward, double right, double up, Json::Value root, bool isGoto) {
    if (isGoto) {
        // Lock to reduce race condition chance
        std::unique_lock<std::mutex> lck(mutex);
        cond_var.wait(lck);
        if (tracking_cmd)
		    cancel_cmd();
	} else {
        track_offboard(OffboardEnum::POSITION_NED, root["north"].asDouble(), root["east"].asDouble(), root["down"].asDouble(), true);
    }

    // Required for logging the target location coordinates, even if executed through position_ned
    lat = root["lat"].asDouble();
    lon = root["lon"].asDouble();
    alt = root["alt"].asDouble();
	this->forward = forward;
	this->right = right;
	this->up = up;
    tracking_cmd = 16;
    publish_cmd_status("move", CommandStatus::START, get_move_coords_node());
}

// Continuously track command completion
void StatusTracker::track() {
	bool published_auto_return = false;
	bool takeoff_status_detected = false;
	bool published_manual = false;
	int takeoff_manual_counter = 0;

	while (true) {
		// Check if the drone was auto disarmed
		if (!manually_disarmed && (telem->armed() ^ was_armed)) {
			if (was_armed) {
				publish_sys_status(SystemStatus::DISARM);
			}
			was_armed = !was_armed;
		}

		// Check if the drone is returning to launch by itself
		if (telem->flight_mode() == Telemetry::FlightMode::ReturnToLaunch && !manually_return) {
			if (!published_auto_return && telem->in_air()) {
				publish_sys_status(SystemStatus::RETURN);
				published_auto_return = true;
			} else if (published_auto_return && !telem->in_air()) {
				published_auto_return = false;
			}
		}

		// If drone is in manual flight mode
		if (static_cast<int>(telem->flight_mode()) > 7) {
			// If currently tracking takeoff, wait 5 seconds (50 * 100 ms wait on this thread) before declaring the
			// command as canceled, as it might have a delay updating the flight mode to takeoff
			if (tracking_cmd == 2) {
				takeoff_manual_counter++;
				if (takeoff_manual_counter > 50) {
					cancel_cmd();
					tracking_cmd = 0;
					publish_sys_status(SystemStatus::START_MANUAL);
					published_manual = true;
				}
			} else if (tracking_cmd > 0){
				cancel_cmd();
				tracking_cmd = 0;
				publish_sys_status(SystemStatus::START_MANUAL);
				published_manual = true;
			} else {
				if (!published_manual) {
					publish_sys_status(SystemStatus::START_MANUAL);
					published_manual = true;
				}
			}
		} else {
			if (published_manual) {
				takeoff_manual_counter = 0;
				published_manual = false;
				publish_sys_status(SystemStatus::STOP_MANUAL);
			}
		}

		// Track current command
		switch (tracking_cmd) {
			// Takeoff command
			case 2:
				if (!takeoff_status_detected && telem->flight_mode() == Telemetry::FlightMode::Takeoff) {
					takeoff_status_detected = true;
				}
				// Finish command if the drone is in air an in hold flight mode
				else if (takeoff_status_detected && telem->flight_mode() != Telemetry::FlightMode::Takeoff) {
					finish_cmd();
					takeoff_status_detected = false;
					// Start go to command if it is queued
					if (queued_goto) {
						queued_goto = false;
						tracking_cmd = 4;
						// Publish status
						publish_cmd_status(tracking_cmd, CommandStatus::START, get_coords_node());
					}
				}
				break;
			// Land command
			case 3:
				// Finish command if the drone is no longer in air
				if (!telem->in_air()) {
					finish_cmd();
				}
				break;
			// Go to command
			case 4:
				// Finish the command if the distance between the current and target positions is less than a threshold
				if (has_reached_goto())
					finish_cmd(get_coords_node());
				break;
			// Return to launch command
			case 5:
				// Finish the command if the drone is no longer in air
				if (!telem->in_air()) {
					finish_cmd();
					manually_return = false;
				}
				break;
			// Position ned command
			case 9:
				if (has_reached_position_ned())
					finish_cmd();
				break;
			// The remaining offboard commands don't have a target goal
			// Turn command
			case 15:
				if (!std::isnan(yaw) && std::abs(toPositiveAngle(telem->attitude_euler().yaw_deg) - toPositiveAngle(yaw)) < 0.5) {
					yaw = NAN;
					finish_cmd();
				}
				break;
			// Move command
            case 16:
                if ((std::isnan(north) && has_reached_goto()) || (!std::isnan(north) && has_reached_position_ned())) {
                    finish_cmd(get_move_coords_node());
                    north = NAN;
                    east = NAN;
                    down = NAN;
                }
                break;
		}
		// Unlock any new command that may have arrived meanwhile and sleep for 100 milliseconds
		cond_var.notify_all();
		usleep(100000);
	}
}

bool StatusTracker::has_reached_goto() {
    // Finish the command if the distance between the current and target positions is less than a threshold
    return (sqrt(pow(distance(lat, lon, telem->position().latitude_deg, telem->position().longitude_deg), 2) +
        pow(telem->position().absolute_altitude_m - alt, 2)) < acceptance_radius());
}

bool StatusTracker::has_reached_position_ned() {
	Telemetry::PositionNed position = telem->position_velocity_ned().position;
    // Finish the command if the NED target and current coordinates distance is less than a threshold
    return (sqrt(pow(position.north_m - north, 2) + pow(position.east_m - east, 2) + pow(position.down_m - down, 2)) < acceptance_radius());
}
