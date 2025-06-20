#include "drone_core/mavsdk_node.hpp"

#include <unistd.h>
#include <math.h>

MavsdkNode::MavsdkNode(rclcpp::Node::SharedPtr base_node, std::string drone_id) {
	this->drone_id = drone_id;
	this->node_ = base_node;

	// Get parameter values (can be set with a config file)
	this->reconnect_mode = node_->get_parameter("reconnectMode").as_int();
	std::string port = node_->get_parameter("port").value_to_string();
	std::string telem_topic = node_->get_parameter("telemetryTopic").value_to_string();
	std::string cmd_topic = node_->get_parameter("commandTopic").value_to_string();
	std::string safe_mode_str = node_->get_parameter("safeMode").value_to_string();
	this->safe_mode = string_to_flight_mode(safe_mode_str);
	bool moveAsGoto = node_->get_parameter("moveAsGoto").as_bool();

	if (!safe_mode_str.empty() && this->safe_mode == Telemetry::FlightMode::Unknown)
	    RCLCPP_WARN(node_->get_logger(), "Provided unknown flight mode as safe mode");

    connect_to_port(port);

	// Print current config
	RCLCPP_INFO(node_->get_logger(), (drone_id + " with tags " + node_->get_parameter("tags").value_to_string() + " ready on port " + port).c_str());
	RCLCPP_INFO(node_->get_logger(), ("Telemetry topic: '" + telem_topic + "', command topic: '" + cmd_topic
				+ "', status topic: '" + node_->get_parameter("statusTopic").value_to_string() + "'").c_str());
	RCLCPP_INFO(node_->get_logger(), ("Telemetry msg rate: once every " + std::to_string(node_->get_parameter("telemetryRateMs").as_int()) + " milliseconds").c_str());
	if (safe_mode_str.empty())
	    RCLCPP_INFO(node_->get_logger(), "No manual safe mode defined");
    else
	    RCLCPP_INFO(node_->get_logger(), ("Safe mode: " + safe_mode_str).c_str());
	if (moveAsGoto)
	    RCLCPP_INFO(node_->get_logger(), "Move commands will be performed as a 'goto' command");
    else
	    RCLCPP_INFO(node_->get_logger(), "Move commands will be performed as a 'position_ned' command");
	if (!node_->get_parameter("ipAddress").value_to_string().empty())
	    RCLCPP_INFO(node_->get_logger(), ("MAC address: " + node_->get_parameter("macAddress").value_to_string() + " IP address: " + node_->get_parameter("ipAddress").value_to_string()).c_str());

	// Initialize Mavsdk
	this->system = mavsdk.systems().at(0);
	this->telem = std::make_shared<Telemetry>(system);
	this->offboard = std::make_shared<Offboard>(system);
	this->action = std::make_shared<Action>(system);

	// Configure default altitudes and max speed
	action->set_maximum_speed(node_->get_parameter("maxSpeed").as_double());
	action->set_return_to_launch_altitude(node_->get_parameter("returnAltitude").as_double());
	action->set_takeoff_altitude(node_->get_parameter("takeoffAltitude").as_double());

	// Configure json builder
	builder["indentation"] = "";
	builder.settings_["precision"] = 10;

	// Create command callback
	auto cmd_callback =
			[this](const typename std_msgs::msg::String::SharedPtr msg) -> void {
				process_cmd(msg->data.c_str());
			};
	// Create command subscriber
	this->cmd_sub = node_->create_subscription<std_msgs::msg::String>(cmd_topic, 10, cmd_callback);

	// Create telemetry messages publisher
	this->telem_pub = node_->create_publisher<std_msgs::msg::String>(telem_topic, 64);
	// Start a thread that periodically retrieves telemetry values
	std::thread msg_repub(&MavsdkNode::publish_telemetry, this);
	msg_repub.detach();

	// Create command watcher and command handler
	this->status_tracker = std::make_shared<StatusTracker>(node_, drone_id, telem, action);
	this->cmd_handler = std::make_shared<CommandHandler>(node_, moveAsGoto, telem, action, offboard, status_tracker);

	// Subscribe to starlink status
	auto starlink_callback =
	[this](const typename std_msgs::msg::String::SharedPtr msg) -> void {
		// print message
		RCLCPP_INFO(node_->get_logger(), ("Starlink status: " + msg->data).c_str());
	};
	RCLCPP_INFO(node_->get_logger(), "Subscribing to starlink status");
	this->starlink_sub = node_->create_subscription<std_msgs::msg::String>("starlink", 64, starlink_callback);

	// Track system timeout
	system->subscribe_is_connected([this](bool is_connected) {
		status_tracker->update_system_connection(is_connected);
		if (!is_connected)
		    handle_exit();
	});

	// Pass Mavsdk logs to rclcpp logger, to be retrievable from rosout
	telem->subscribe_status_text([this](Telemetry::StatusText status) {
		switch(status.type) {
			case (Telemetry::StatusTextType::Info):
			case (Telemetry::StatusTextType::Notice):
				RCLCPP_INFO(node_->get_logger(), ("MAVSDK log: " + status.text).c_str());
				break;
			case (Telemetry::StatusTextType::Warning):
				RCLCPP_WARN(node_->get_logger(), ("MAVSDK log: " + status.text).c_str());
				break;
			case (Telemetry::StatusTextType::Critical):
			case (Telemetry::StatusTextType::Error):
			case (Telemetry::StatusTextType::Alert):
			case (Telemetry::StatusTextType::Emergency):
				RCLCPP_ERROR(node_->get_logger(), ("MAVSDK log: " + status.text).c_str());
				break;
			case (Telemetry::StatusTextType::Debug):
				RCLCPP_DEBUG(node_->get_logger(), ("MAVSDK log: " + status.text).c_str());
				break;
		}
	});
}

void MavsdkNode::handle_exit() {
    if (reconnect_mode == 0)
        exit(1);
    else if (reconnect_mode == 1)
        exit(5);
}

void MavsdkNode::connect_to_port(std::string port) {
    // Attempt to connect to a device at the given port
	if (mavsdk.add_any_connection(port) != ConnectionResult::Success) {
	    // If attempting to connect to serial port
	    if (port.rfind("serial", 0) == 0)
            RCLCPP_ERROR(node_->get_logger(),
                ("No device found on port \'" + port + "\'. Is the flight controller connected to that port?").c_str());
        else
            RCLCPP_ERROR(node_->get_logger(),
                ("Unable to connect to port \'" + port + "\'. Is the port closed?").c_str());
        if (reconnect_mode == 2) {
            while (mavsdk.add_any_connection(port) != ConnectionResult::Success) {
                RCLCPP_INFO(node_->get_logger(), ("Attempting to reconnect to port \'" + port + "\'...").c_str());
                usleep(1000000);
            }
        } else {
            handle_exit();
        }
	}
	// Attempt to find a mavlink system on the given port
	int timeout_counter = 0;
	bool reconnecting = false;
	while (mavsdk.systems().size() == 0) {
        if (timeout_counter > 20) {
            if (!reconnecting) {
                if (port.rfind("serial", 0) == 0)
                    RCLCPP_ERROR(node_->get_logger(),
                        ("Connected to port \'" + port + "\', but timed out waiting for a Mavlink system. Is the flight controller properly configured?").c_str());
                else
                    RCLCPP_ERROR(node_->get_logger(),
                        ("Connected to port \'" + port + "\', but timed out waiting for a Mavlink system. Is the simulator running on that port?").c_str());
            }
            if (reconnect_mode == 2) {
                RCLCPP_INFO(node_->get_logger(), ("Searching for Mavlink system on port \'" + port + "\'...").c_str());
                timeout_counter = -1;
                reconnecting = true;
            } else {
                handle_exit();
            }
        }
        timeout_counter++;
        usleep(100000);
	}
}

// Retrieve telemetry and publish to a topic
void MavsdkNode::publish_telemetry() {
	Telemetry::Position pos;
	Telemetry::Position home;
	Telemetry::Battery battery;
	Telemetry::Imu imu;
	Telemetry::VelocityNed velocity;
	Telemetry::PositionNed position;
	Telemetry::GpsInfo gps_info;
	Json::Value root;

	while (std::isnan(telem->battery().voltage_v))
		usleep(10000);

	while (rclcpp::ok()) {
		if (system->is_connected()) {
			pos = telem->position();
			home = telem->home();
			battery = telem->battery();
			imu = telem->imu();
			velocity = telem->velocity_ned();
			position = telem->position_velocity_ned().position;
			gps_info = telem->gps_info();

			// Set the message fields
			root["droneId"] = drone_id;
			root["timestamp"] = get_timestamp();
			root["position"]["lat"] = pos.latitude_deg;
			root["position"]["lon"] = pos.longitude_deg;
			root["position"]["alt"] = pos.absolute_altitude_m;
			root["position"]["height"] = pos.relative_altitude_m;
			root["home"]["lat"] = home.latitude_deg;
			root["home"]["lon"] = home.longitude_deg;
			root["home"]["alt"] = home.absolute_altitude_m;
			root["home"]["height"] = home.relative_altitude_m;
			root["landed_state"] = landed_state_to_string(telem->landed_state());
			root["armed"] = telem->armed();
			root["battery"]["voltage"] = battery.voltage_v;
			root["battery"]["remaining_percent"] = battery.remaining_percent;
			root["flight_mode"] = flight_mode_to_string(telem->flight_mode());
			root["heading"] = telem->attitude_euler().yaw_deg;
			root["healthFail"] = health_to_json(telem->health());
			root["speed"] = sqrt(pow(velocity.down_m_s, 2) + pow(velocity.north_m_s, 2) + pow(velocity.east_m_s, 2));
			root["gpsInfo"]["satellites"] = gps_info.num_satellites;
			root["gpsInfo"]["fixType"] =  gps_fix_to_string(gps_info.fix_type);
			root["imu"]["accel"]["forward"] = imu.acceleration_frd.forward_m_s2;
			root["imu"]["accel"]["right"] = imu.acceleration_frd.right_m_s2;
			root["imu"]["accel"]["down"] = imu.acceleration_frd.down_m_s2;
			root["imu"]["ang_vel"]["forward"] = imu.angular_velocity_frd.forward_rad_s;
			root["imu"]["ang_vel"]["right"] = imu.angular_velocity_frd.right_rad_s;
			root["imu"]["ang_vel"]["down"] = imu.angular_velocity_frd.down_rad_s;
			root["position_ned"]["north"] = position.north_m;
			root["position_ned"]["east"] = position.east_m;
			root["position_ned"]["down"] = position.down_m;
			root["velocity_ned"]["north"] = velocity.north_m_s;
			root["velocity_ned"]["east"] = velocity.east_m_s;
			root["velocity_ned"]["down"] = velocity.down_m_s;
			root["height"] = telem->distance_sensor().current_distance_m;

			// Create and publish ros message
			auto message = std_msgs::msg::String();
			message.data = Json::writeString(builder, root);
			publish_json(root, builder, telem_pub);
		}
		usleep(node_->get_parameter("telemetryRateMs").as_int() * 1000);
	}
}

// Mavsdk command executor
void MavsdkNode::process_cmd(std::string msg) {
	Json::Value root;
	Json::Reader reader;

	// Invalid Json message
	if (!reader.parse(msg, root))
		RCLCPP_ERROR(node_->get_logger(), "Received a command that can't be parsed.");

	// Confirm the message is for this drone
	if (root["droneId"] != drone_id) {
		return;
	}

	if (!root.isMember("cmd")) {
		RCLCPP_ERROR(node_->get_logger(), "Command must be defined.");
		return;
	}

	if (!can_run_cmd(root["cmd"].asString()))
		return;

	// Check the command mode
	if (root["mode"] == "action") {
		ActionEnum action_cmd = string_to_action(root["cmd"].asString());
		handle_action_cmd(action_cmd, root);
	} else if (root["mode"] == "offboard") {
		OffboardEnum offboard_cmd = string_to_offboard(root["cmd"].asString());
		handle_offboard_cmd(offboard_cmd, root);
	} else if (root["mode"] == "custom") {
		CustomCmdEnum custom_cmd = string_to_custom_cmd(root["cmd"].asString());
		handle_custom_cmd(custom_cmd, root);
	} else {
		if (!root.isMember("mode")) {
			RCLCPP_ERROR(node_->get_logger(), "Command must have a mode.");
			status_tracker->track_cmd_failure(root["cmd"].asString(), "missing \"mode\"");
		}
		else
			RCLCPP_ERROR(node_->get_logger(), ("Command mode \"" + root["mode"].asString() + "\" does not exist."
											  + "Valid modes are \"action\" and \"offboard\".").c_str());
			status_tracker->track_cmd_failure(root["cmd"].asString(), "mode \"" + root["mode"].asString() + "\" does not exist");
	}
}

bool MavsdkNode::can_run_cmd(std::string cmd) {
    if (safe_mode != Telemetry::FlightMode::Unknown && safe_mode == telem->flight_mode()) {
		RCLCPP_ERROR(node_->get_logger(), ("Command " + cmd + " failed: safe mode '" + flight_mode_to_string(safe_mode) + "' is active").c_str());
		status_tracker->track_cmd_failure(cmd, "safe flight mode is active");
		return false;
    }
	ActionEnum action_cmd = string_to_action(cmd);
	if (!telem->armed() && action_cmd != ActionEnum::ARM) {
		RCLCPP_ERROR(node_->get_logger(), ("Command " + cmd + " failed: disarmed").c_str());
		status_tracker->track_cmd_failure(cmd, "disarmed");
		return false;
	}
	return true;
}

// Execute action command
void MavsdkNode::handle_action_cmd(ActionEnum action_cmd, Json::Value root) {
	// print action_cmd
	switch (action_cmd) {
		case ActionEnum::ARM:
			cmd_handler->arm();
			break;
		case ActionEnum::DISARM:
			cmd_handler->disarm();
			break;
		case ActionEnum::TAKEOFF:
			cmd_handler->takeoff(root);
			break;
		case ActionEnum::LAND:
			cmd_handler->land();
			break;
		case ActionEnum::RETURN:
			cmd_handler->return_to_launch(root);
			break;
		case ActionEnum::GOTO:
			cmd_handler->go_to(root);
			break;
		case ActionEnum::GEOFENCE:
			cmd_handler->go_to_geofence(root);
			break;
		default:
			RCLCPP_ERROR(node_->get_logger(), ("Action command \"" + root["cmd"].asString() + "\" is not supported.").c_str());
			return;
	}
}

// Execute custom command
void MavsdkNode::handle_custom_cmd(CustomCmdEnum custom_cmd, Json::Value root) {
	switch (custom_cmd) {
		case CustomCmdEnum::TURN:
			cmd_handler->turn(root);
			break;
		case CustomCmdEnum::MOVE:
			cmd_handler->move(root);
			break;
		case CustomCmdEnum::CANCEL:
			cmd_handler->cancel();
			break;
		default:
			RCLCPP_ERROR(node_->get_logger(), ("Custom command \"" + root["cmd"].asString() + "\" is not supported.").c_str());
			return;
	}
}

// Execute offboard command
void MavsdkNode::handle_offboard_cmd(OffboardEnum offboard_cmd, Json::Value root) {
	switch (offboard_cmd) {
		case OffboardEnum::START:
			cmd_handler->start_offb();
			return;
		case OffboardEnum::STOP:
			cmd_handler->stop_offb();
			return;
		case OffboardEnum::POSITION_NED:
			cmd_handler->position_ned(root);
			break;
		case OffboardEnum::VELOCITY_NED:
			cmd_handler->velocity_ned(root);
			break;
		case OffboardEnum::VELOCITY_BODY:
			cmd_handler->velocity_body(root);
			break;
		case OffboardEnum::ATTITUDE:
			cmd_handler->attitude(root);
			break;
		case OffboardEnum::ATTITUDE_RATE:
			cmd_handler->attitude_rate(root);
			break;
		case OffboardEnum::ACTUATOR_CONTROL:
			cmd_handler->actuator_control(root);
			break;
		default:
			RCLCPP_ERROR(node_->get_logger(), ("Offboard command \"" + root["cmd"].asString() + "\" is not supported").c_str());
			return;
	}
}
