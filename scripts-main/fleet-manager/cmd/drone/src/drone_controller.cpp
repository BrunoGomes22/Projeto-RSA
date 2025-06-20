#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include "drone_core/mavsdk_node.hpp"

#include "drone_core/restrictions/geofence.hpp"

using namespace std::chrono_literals;

void signalHandler( int signum ) {
   exit(signum);
}

int main(int argc, char *argv[]) {
	signal(SIGINT, signalHandler);

	// Clear stdout buffer
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	// Pass command line arguments to rclcpp.
	rclcpp::init(argc, argv);

	// Read drone id/node name from the command line arguments
	std::string node_name;
	for (int i = 0; i < argc; i++) {
		if (std::string(argv[i]).compare("-n") == 0) {
			if (i == argc - 1) {
				std::cerr << "Error: no node name specified after \'-n\'." << std::endl;
				return 1;
			}
			if (argv[i + 1][0] == '-') {
				std::cerr << "Error: found flag \'" << argv[i + 1] << "\' instead of node name." << std::endl;
				return 1;
			}
			node_name = argv[i + 1];
			break;
		}
	}
	if (node_name.size() == 0) {
		std::cerr << "Error: no node name was provided. Add the \'-n\' flag followed by the desired node name."
				  << std::endl;
		return 1;
	}

	// TODO rmw QoS settings

	// Initialize ROS2 node
	auto node = rclcpp::Node::make_shared(node_name);
	rclcpp::Logger logger_ = node->get_logger();

	// Check if a node with the same name already exists
	auto existing_nodes = node->get_node_names();
	if (std::count(existing_nodes.begin(), existing_nodes.end(), node_name) > 1) {
		std::cerr << "Error: Node " << node_name << " already exists!" << std::endl;
		return 1;
	}

	// Create parameter descriptor that sets the parameters as read-only
    rcl_interfaces::msg::ParameterDescriptor param_descriptor;
    param_descriptor.read_only = true;

    // Declare parameters and default values
    std::vector<std::string> default_tag {"simulated"};
    node->declare_parameter("port", "udp://:14540", param_descriptor);
    node->declare_parameter("telemetryTopic", "/telem", param_descriptor);
    node->declare_parameter("commandTopic", "/cmd", param_descriptor);
    node->declare_parameter("statusTopic", "/status", param_descriptor);
    node->declare_parameter("paramTopic", "/param", param_descriptor);
    node->declare_parameter("assignable", true);
    node->declare_parameter("telemetryRateMs", 500);
    node->declare_parameter("acceptanceRadius", 0.5);
    node->declare_parameter("takeoffAltitude", 15.0);
    node->declare_parameter("returnAltitude", 15.0);
    node->declare_parameter("maxSpeed", 5.0);
    node->declare_parameter("tags", default_tag);
    node->declare_parameter("ipAddress", "", param_descriptor);
    node->declare_parameter("macAddress", "", param_descriptor);
    node->declare_parameter("reconnectMode", 2, param_descriptor);
    node->declare_parameter("safeMode", "", param_descriptor);
    node->declare_parameter("moveAsGoto", false, param_descriptor);
	node->declare_parameter("maxAltitude", 10.0, param_descriptor);

	auto mav_node = std::make_shared<MavsdkNode>(node, node_name);

	// Do some work when available
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
