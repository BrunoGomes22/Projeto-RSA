#ifndef FLEETMAN_DRONE_MAVSDK_HANDLER
#define FLEETMAN_DRONE_MAVSDK_HANDLER

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>

#include <jsoncpp/json/json.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "drone_core/cmd_handler.hpp"
#include "drone_core/drone_utils.hpp"
#include "drone_core/status_tracker.hpp"

using namespace drone_utils;
using namespace coord_utils;
using namespace mavsdk;

class MavsdkNode {
public:
	MavsdkNode(rclcpp::Node::SharedPtr base_node, std::string drone_id);

private:
	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telem_pub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr starlink_sub;

	std::shared_ptr<StatusTracker> status_tracker;
	std::shared_ptr<CommandHandler> cmd_handler;

	std::string drone_id;
	uint8_t reconnect_mode;
    Telemetry::FlightMode safe_mode;

	Mavsdk mavsdk;
	std::shared_ptr<System> system;
	Json::StreamWriterBuilder builder;
	std::shared_ptr <Telemetry> telem;
	std::shared_ptr <Offboard> offboard;
	std::shared_ptr <Action> action;

    void connect_to_port(std::string port);
    void handle_exit();
	void process_cmd(std::string msg);
	void publish_telemetry();
	bool can_run_cmd(std::string cmd);

	void handle_offboard_cmd(OffboardEnum offboard_cmd, Json::Value root);
	void handle_action_cmd(ActionEnum action_cmd, Json::Value root);
	void handle_custom_cmd(CustomCmdEnum custom_cmd, Json::Value root);
};

#endif  // FLEETMAN_DRONE_MAVSDK_HANDLER