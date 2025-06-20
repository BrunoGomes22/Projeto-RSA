#ifndef FLEETMAN_DRONE_STATUS_TRACKER_H
#define FLEETMAN_DRONE_STATUS_TRACKER_H

#include <condition_variable>
#include <mutex>
#include <unordered_map>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <jsoncpp/json/json.h>

#include "drone_core/drone_utils.hpp"
#include "drone_core/coord_utils.hpp"

using namespace drone_utils;
using namespace coord_utils;
using namespace std::chrono;

class StatusTracker {
public:
	StatusTracker(rclcpp::Node::SharedPtr node_, std::string drone_id, std::shared_ptr<Telemetry> telem, std::shared_ptr<Action>);
	void update_system_connection(bool is_connected);
	void track_cmd_failure(std::string cmd, std::string error);
	void track_start_stop_offb(bool start, bool silenced=false);
	void track_arm_disarm(bool is_disarm);
	void track_action(ActionEnum cmd);
	void track_goto(double lat, double lon, float alt);
	void track_offboard(OffboardEnum cmd, float north=NAN, float east=NAN, float down=NAN, bool isMove=false);
	void track_turn(float yaw, float relative_deg);
	void track_move(double forward, double right, double up, Json::Value root, bool isGoto);
	void publish_custom_status(CustomCmdEnum cmd, std::string status);

private:
	enum class CommandStatus {
		START = 0,
		STOP,
		CANCEL,
		FINISH,
		QUEUED,
		SUCCESS,
		FAILURE
	};

	enum class SystemStatus {
		CONNECT = 0,
		DISCONNECT,
		DISARM,
		RETURN,
		STOP_OFFBOARD,
		START_MANUAL,
		STOP_MANUAL,
	};

	static const std::unordered_map <CommandStatus, std::string> cmd_status_table;
	static const std::unordered_map <SystemStatus, std::string> sys_status_table;

	std::string cmd_status_to_string(CommandStatus status);
	std::string sys_status_to_string(SystemStatus status);

	std::string drone_id;
	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
	std::shared_ptr<Telemetry> telem;
	std::shared_ptr<Action> action;
	Json::StreamWriterBuilder builder;

	Json::Value get_coords_node(double lat=NAN, double lon=NAN, float alt=NAN);
	Json::Value get_move_coords_node();
	Json::Value get_base_message();
	void publish_cmd_status(std::string cmd, CommandStatus status, Json::Value value=Json::nullValue);
	void publish_cmd_status(int cmd, CommandStatus status, Json::Value value=Json::nullValue);
	void publish_cmd_status(ActionEnum cmd, CommandStatus status, Json::Value value=Json::nullValue);
	void publish_cmd_status(OffboardEnum cmd, CommandStatus status, Json::Value value=Json::nullValue);
	void publish_sys_status(SystemStatus status);
	void cancel_cmd();
	void dequeue_goto();
	void finish_cmd(Json::Value value=Json::nullValue);
	void track();
	double acceptance_radius();

	bool has_reached_goto();
    bool has_reached_position_ned();

	bool was_armed;
	bool manually_disarmed;
	bool manually_return;
	int tracking_cmd;
	bool queued_goto;
	double lat, lon;
	float alt;
	float north, east, down;
	float yaw;
	double forward, right, up; // Only used for publishing move finished status message

	std::mutex mutex;
	std::condition_variable cond_var;
};
#endif //FLEETMAN_DRONE_STATUS_TRACKER_H
