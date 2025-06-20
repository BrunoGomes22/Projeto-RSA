#include "drone_core/drone_utils.hpp"

#include <unordered_map>

using namespace std::chrono;

namespace drone_utils {
	// Supported action commands
	static std::unordered_map <std::string, ActionEnum> const str_action_table = {
			{"arm",     ActionEnum::ARM},
			{"disarm",  ActionEnum::DISARM},
			{"takeoff", ActionEnum::TAKEOFF},
			{"land",    ActionEnum::LAND},
			{"goto",    ActionEnum::GOTO},
			{"return",  ActionEnum::RETURN},
			{"geofence",ActionEnum::GEOFENCE}
	};

	// Supported offboard commands
	static std::unordered_map <std::string, OffboardEnum> const str_offb_table = {
			{"start_offboard",   OffboardEnum::START},
			{"stop_offboard",    OffboardEnum::STOP},
			{"position_ned",     OffboardEnum::POSITION_NED},
			{"velocity_ned",     OffboardEnum::VELOCITY_NED},
			{"velocity_body",    OffboardEnum::VELOCITY_BODY},
			{"attitude",         OffboardEnum::ATTITUDE},
			{"attitude_rate",    OffboardEnum::ATTITUDE_RATE},
			{"actuator_control", OffboardEnum::ACTUATOR_CONTROL}
	};

	// Supported custom commands
	static std::unordered_map <std::string, CustomCmdEnum> const str_custom_table = {
			{"turn",			CustomCmdEnum::TURN},
			{"move", 			CustomCmdEnum::MOVE},
			{"cancel",			CustomCmdEnum::CANCEL}
	};


	// Flight modes from string
	static std::unordered_map <std::string, Telemetry::FlightMode> const str_flight_mode_table = {
        {"unknown",             Telemetry::FlightMode::Unknown},
        {"ready",               Telemetry::FlightMode::Ready},
        {"takeoff",             Telemetry::FlightMode::Takeoff},
        {"hold",                Telemetry::FlightMode::Hold},
        {"mission",             Telemetry::FlightMode::Mission},
        {"return_to_launch",    Telemetry::FlightMode::ReturnToLaunch},
        {"land",                Telemetry::FlightMode::Land},
        {"offboard",            Telemetry::FlightMode::Offboard},
        {"follow_me",           Telemetry::FlightMode::FollowMe},
        {"manual",              Telemetry::FlightMode::Manual},
        {"altctl",              Telemetry::FlightMode::Altctl},
        {"posctl",              Telemetry::FlightMode::Posctl},
        {"acro",                Telemetry::FlightMode::Acro},
        {"stabilized",          Telemetry::FlightMode::Stabilized},
        {"rattitude",           Telemetry::FlightMode::Rattitude}
	};

	// Conversion from action enum to string
	static std::unordered_map <ActionEnum, std::string> const action_str_table = {
			{ActionEnum::ARM,		"arm"},
			{ActionEnum::DISARM,	"disarm"},
			{ActionEnum::TAKEOFF,	"takeoff"},
			{ActionEnum::LAND,		"land"},
			{ActionEnum::GOTO,		"goto"},
			{ActionEnum::RETURN,	"return"},
			{ActionEnum::GEOFENCE,	"geofence"}
	};

	// Conversion from offboard enum to string
	static std::unordered_map <OffboardEnum, std::string> const offb_str_table = {
			{OffboardEnum::START,				"start_offboard",},
			{OffboardEnum::STOP,				"stop_offboard"},
			{OffboardEnum::POSITION_NED,		"position_ned"},
			{OffboardEnum::VELOCITY_NED,		"velocity_ned"},
			{OffboardEnum::VELOCITY_BODY,		"velocity_body"},
			{OffboardEnum::ATTITUDE,			"attitude"},
			{OffboardEnum::ATTITUDE_RATE,		"attitude_rate"},
			{OffboardEnum::ACTUATOR_CONTROL,	"actuator_control"}
	};

	// Conversion from custom cmd enum to string
	static std::unordered_map <CustomCmdEnum, std::string> const custom_str_table = {
			{CustomCmdEnum::TURN,	"turn"},
			{CustomCmdEnum::MOVE,	"move"},
			{CustomCmdEnum::CANCEL,	"cancel"}
	};
	
	// Conversion from flight mode to string
	static std::unordered_map <Telemetry::FlightMode, std::string> const flight_mode_str_table = {
        {Telemetry::FlightMode::Unknown,        "unknown"},
        {Telemetry::FlightMode::Ready,          "ready"},
        {Telemetry::FlightMode::Takeoff,        "takeoff"},
        {Telemetry::FlightMode::Hold,           "hold"},
        {Telemetry::FlightMode::Mission,        "mission"},
        {Telemetry::FlightMode::ReturnToLaunch, "return_to_launch"},
        {Telemetry::FlightMode::Land,           "land"},
        {Telemetry::FlightMode::Offboard,       "offboard"},
        {Telemetry::FlightMode::FollowMe,       "follow_me"},
        {Telemetry::FlightMode::Manual,         "manual"},
        {Telemetry::FlightMode::Altctl,         "altctl"},
        {Telemetry::FlightMode::Posctl,         "posctl"},
        {Telemetry::FlightMode::Acro,           "acro"},
        {Telemetry::FlightMode::Stabilized,     "stabilized"},
        {Telemetry::FlightMode::Rattitude,      "rattitude"}
	};

	// Convert string to action enum
	ActionEnum string_to_action(std::string str) {
		if (auto it = str_action_table.find(str); it != str_action_table.end())
			return it->second;
		return ActionEnum::UNKNOWN;
	}

	// Convert string to offboard enum
	OffboardEnum string_to_offboard(std::string str) {
		if (auto it = str_offb_table.find(str); it != str_offb_table.end())
			return it->second;
		return OffboardEnum::UNKNOWN;
	}

	// Convert string to custom cmd enum
	CustomCmdEnum string_to_custom_cmd(std::string str) {
		if (auto it = str_custom_table.find(str); it != str_custom_table.end())
			return it->second;
		return CustomCmdEnum::UNKNOWN;
	}

	// Convert enum to string
	std::string enum_to_string(int val) {
		if (val < 7) {
			ActionEnum action = static_cast<ActionEnum>(val);
			if (auto it = action_str_table.find(action); it != action_str_table.end())
				return it->second;
		} else if (val < 15) {
			OffboardEnum offboard = static_cast<OffboardEnum>(val);
			if (auto it = offb_str_table.find(offboard); it != offb_str_table.end())
				return it->second;
		} else {
			CustomCmdEnum custom = static_cast<CustomCmdEnum>(val);
			if (auto it = custom_str_table.find(custom); it != custom_str_table.end())
				return it->second;
		}
		return nullptr;
	}

	std::string enum_to_string(ActionEnum action) {
		if (auto it = action_str_table.find(action); it != action_str_table.end())
			return it->second;
		return nullptr;
	}

	std::string enum_to_string(OffboardEnum offboard) {
		if (auto it = offb_str_table.find(offboard); it != offb_str_table.end())
			return it->second;
		return nullptr;
	}

	std::string enum_to_string(CustomCmdEnum custom) {
		if (auto it = custom_str_table.find(custom); it != custom_str_table.end())
			return it->second;
		return nullptr;
	}

	// Get current timestamp
	Json::Int64 get_timestamp() {
		return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	}

	// Publish json string as a ROS2 message
	void publish_json(Json::Value root, Json::StreamWriterBuilder builder,
			rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub) {
		// Create and publish ros message
		auto message = std_msgs::msg::String();
		message.data = Json::writeString(builder, root);
		pub->publish(message);
	}

	// Convert Mavsdk Health struct to a Json list
	Json::Value health_to_json(Telemetry::Health health) {
		Json::Value failure(Json::arrayValue);
		if (!health.is_gyrometer_calibration_ok)
			failure.append("gyrometer");
		if (!health.is_accelerometer_calibration_ok)
			failure.append("accelerometer");
		if (!health.is_magnetometer_calibration_ok)
			failure.append("magnetometer");
		if (!health.is_level_calibration_ok)
			failure.append("level");
		if (!health.is_local_position_ok)
			failure.append("localPos");
		if (!health.is_global_position_ok)
			failure.append("globalPos");
		if (!health.is_home_position_ok)
			failure.append("homePos");

		return failure;
	}

	// Convert GPS fix type enum to a string
	std::string gps_fix_to_string(Telemetry::FixType ft) {
		std::string fix;
		switch (ft) {
			case Telemetry::FixType::NoGps:
				fix = "no_gps";
				break;
			case Telemetry::FixType::NoFix:
				fix = "no_fix";
				break;
			case Telemetry::FixType::Fix2D:
				fix = "fix_2d";
				break;
			case Telemetry::FixType::Fix3D:
				fix = "fix_3d";
				break;
			case Telemetry::FixType::FixDgps:
				fix = "fix_dgps";
				break;
			case Telemetry::FixType::RtkFloat:
				fix = "rtk_float";
				break;
			case Telemetry::FixType::RtkFixed:
				fix = "rtk_fixed";
				break;
		}
		return fix;
	}

	// Convert Mavsdk landed state enum to a string
	std::string landed_state_to_string(Telemetry::LandedState ls) {
		std::string state;
		switch (ls) {
			case Telemetry::LandedState::Unknown:
				state = "unknown";
				break;
			case Telemetry::LandedState::OnGround:
				state = "on_ground";
				break;
			case Telemetry::LandedState::InAir:
				state = "in_air";
				break;
			case Telemetry::LandedState::TakingOff:
				state = "taking_off";
				break;
			case Telemetry::LandedState::Landing:
				state = "landing";
				break;
		}
		return state;
	}

	// Convert Mavsdk flight mode enum to a string
	std::string flight_mode_to_string(Telemetry::FlightMode fm) {
        if (auto it = flight_mode_str_table.find(fm); it != flight_mode_str_table.end())
            return it->second;
		return nullptr;
	}

	// Convert string to a Mavsdk flight mode enum
	Telemetry::FlightMode string_to_flight_mode(std::string fm) {
		if (auto it = str_flight_mode_table.find(fm); it != str_flight_mode_table.end())
            return it->second;
        return Telemetry::FlightMode::Unknown;
	}
}
