#ifndef FLEETMAN_DRONE_DRONE_UTILS_H
#define FLEETMAN_DRONE_DRONE_UTILS_H

#include <jsoncpp/json/json.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace mavsdk;

namespace drone_utils {
	enum class ActionEnum {
		ARM = 0,
		DISARM,
		TAKEOFF,
		LAND,
		GOTO,
		RETURN,
		UNKNOWN,
		GEOFENCE
	};

	enum class OffboardEnum {
		START = 7,
		STOP,
		POSITION_NED,
		VELOCITY_NED,
		VELOCITY_BODY,
		ATTITUDE,
		ATTITUDE_RATE,
		ACTUATOR_CONTROL,
		UNKNOWN
	};

	enum class CustomCmdEnum {
		TURN = 15,
		MOVE,
		CANCEL,
		UNKNOWN
	};

	ActionEnum string_to_action(std::string str);
	OffboardEnum string_to_offboard(std::string str);
	CustomCmdEnum string_to_custom_cmd(std::string str);
	std::string enum_to_string(int val);
	std::string enum_to_string(ActionEnum action);
	std::string enum_to_string(OffboardEnum offboard);
	std::string enum_to_string(CustomCmdEnum custom);

	Json::Int64 get_timestamp();
	void publish_json(Json::Value root, Json::StreamWriterBuilder builder,
			rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub);

	Json::Value health_to_json(Telemetry::Health health);
	std::string gps_fix_to_string(Telemetry::FixType ft);
	std::string landed_state_to_string(Telemetry::LandedState ls);
	std::string flight_mode_to_string(Telemetry::FlightMode fm);
	Telemetry::FlightMode string_to_flight_mode(std::string fm);
}

#endif //FLEETMAN_DRONE_DRONE_UTILS_H
