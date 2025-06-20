#ifndef FLEETMAN_DRONE_CMD_HANDLER_HPP
#define FLEETMAN_DRONE_CMD_HANDLER_HPP

#include <jsoncpp/json/json.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <rclcpp/rclcpp.hpp>

#include "drone_core/status_tracker.hpp"
#include "drone_core/restrictions/restrictions.hpp"

class CommandHandler {
public:
	CommandHandler(rclcpp::Node::SharedPtr node_,
	              bool move_as_goto,
				  std::shared_ptr<Telemetry> telem,
				  std::shared_ptr<Action> action,
				  std::shared_ptr<Offboard> offboard,
				  std::shared_ptr<StatusTracker> tracker);
	void arm();
	void disarm();
	void takeoff(Json::Value root);
	void land();
	void go_to(Json::Value root, bool is_move=false);
	void return_to_launch(Json::Value root);
	void go_to_geofence(Json::Value root);
	void start_offb(bool silenced=false);
	void stop_offb(bool silenced=false);
	void position_ned(Json::Value root, bool is_move=false);
	void velocity_ned(Json::Value root, bool is_turn=false);
	void velocity_body(Json::Value root);
	void attitude(Json::Value root);
	void attitude_rate(Json::Value root);
	void actuator_control(Json::Value root);
	void turn(Json::Value root);
	void move(Json::Value root);
	void cancel();

private:
	rclcpp::Node::SharedPtr node_;
	std::shared_ptr<Telemetry> telem;
	std::shared_ptr<Action> action;
	std::shared_ptr<Offboard> offboard;
	std::shared_ptr<StatusTracker> tracker;
	std::shared_ptr<RestrictionsChecker> checker;
	float last_target_alt;
	float last_target_height;
	bool move_as_goto;

	float takeoff_alt();
	float return_alt();
	float max_speed();
	float max_alt();
	void handle_action_cmd_error(Action::Result action_result, std::string cmd);
	void handle_offboard_cmd_error(Offboard::Result offb_result, std::string cmd);
	bool can_move(std::string cmd, Json::Value root);
};

#endif //FLEETMAN_DRONE_CMD_HANDLER_HPP
