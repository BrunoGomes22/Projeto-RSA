package org.nap.fleetman.server.core;

import org.ros2.rcljava.publisher.Publisher;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

/**
 * Publish ROS2 messages
 */
@Service
public class MessagePublisher {
	private final Publisher<std_msgs.msg.String> cmdPub;
	private final Publisher<std_msgs.msg.String> infoPub;
	private final Publisher<std_msgs.msg.UInt64> heartbeatPub;

	public MessagePublisher(GroundNode node,
							@Value("${fleetman.ros2.topic.cmd:cmd}") String cmdTopic,
							@Value("${fleetman.ros2.topic.info:info}") String infoTopic,
							@Value("${fleetman.ros2.topic.hearbeat:heartbeat}") String heartbeatTopic) {
		cmdPub = node.getNode().createPublisher(std_msgs.msg.String.class, cmdTopic);
		infoPub = node.getNode().createPublisher(std_msgs.msg.String.class, infoTopic);
		heartbeatPub = node.getNode().createPublisher(std_msgs.msg.UInt64.class, heartbeatTopic);
	}

	// Publish command to the topic as-is, with the addition of the droneId.
	// There's a single command topic for all drones, so the messages include
	// the drone identifier in order to be distinguishable.
	public void publishCommand(String cmd) {
		std_msgs.msg.String rosMessage = new std_msgs.msg.String();
		rosMessage.setData(cmd);
		cmdPub.publish(rosMessage);
	}

	// Publish assorted data when requested through a mission script
	public void publishInfo(String msg) {
		std_msgs.msg.String rosMessage = new std_msgs.msg.String();
		rosMessage.setData(msg);
		infoPub.publish(rosMessage);
	}

	// Publish a ground station heartbeat message
	public void publishHeartbeat(long timestamp) {
		std_msgs.msg.UInt64 rosMessage = new std_msgs.msg.UInt64();
		rosMessage.setData(timestamp);
		heartbeatPub.publish(rosMessage);
	}
}
