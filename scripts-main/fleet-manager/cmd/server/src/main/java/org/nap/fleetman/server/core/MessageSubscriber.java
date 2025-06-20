package org.nap.fleetman.server.core;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.eclipse.paho.client.mqttv3.IMqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.nap.fleetman.server.api.NotFoundException;
import org.nap.fleetman.server.remote.RemoteTaskHandler;
import org.nap.fleetman.server.model.drone.CommandEnum;
import org.nap.fleetman.server.model.drone.StatusMessage;
import org.nap.fleetman.server.model.telemetry.Telemetry;
import org.nap.fleetman.server.parameters.ParameterTracker;
import org.nap.fleetman.server.sensors.SensorManager;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.BeanCreationNotAllowedException;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import rcl_interfaces.msg.ParameterEvent;

import java.util.HashMap;


/**
 * Create ROS2 message subscriptions
 * NOTE: only one thread is created per submission, which means if multiple drones send a message at about the same
 * time, one message will only be processed after the previous one is done.
 */
@Service
public class MessageSubscriber {
	private static final Logger log = LoggerFactory.getLogger(MessageSubscriber.class);

	@Value("${fleetman.mission.printStackTrace:true}")
	private boolean printFullStackTrace;
	private final StatusUpdater statusUpdater;
	private final SensorManager sensorMan;
	private final RemoteTaskHandler remoteTaskHandler;
	private final ObjectMapper mapper;

	public MessageSubscriber(GroundNode node, StatusUpdater statusUpdater, SensorManager sensorMan, RemoteTaskHandler remoteTaskHandler, ParameterTracker paramTracker,
							 @Value("${fleetman.ros2.topic.telem:telem}") String telemTopic,
							 @Value("${fleetman.ros2.topic.sensor:sensors}") String sensorTopic,
							 @Value("${fleetman.ros2.topic.mission:mission_status}") String missionTopic,
							 @Value("${fleetman.ros2.topic.status:status}") String statusTopic,
							 IMqttClient mqttClient) {
		this.statusUpdater = statusUpdater;
		this.sensorMan = sensorMan;
		this.remoteTaskHandler = remoteTaskHandler;
		mapper = new ObjectMapper();
		mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

		// Subscribe to mqtt sensor messages
		try {
			if (mqttClient != null && mqttClient.isConnected()) {
				mqttClient.subscribe(sensorTopic, (topic, msg) -> {
					handleSensorMessage(new String(msg.getPayload()));
				});
				log.info("Connected to mqtt server at " + mqttClient.getServerURI());
			}
		} catch (MqttException e) {
			log.warn("Unexpected exception caught mqtt client");
			e.printStackTrace();
		}

		// Create telemetry subscription
		node.getNode().createSubscription(
				std_msgs.msg.String.class, telemTopic, msg -> {
					try {
						handleTelemMessage(msg.getData());
					} catch (BeanCreationNotAllowedException ignored) {
					} catch (Exception e) {
						log.warn("Unexpected exception caught in method called by telemetry subscriber");
						if (printFullStackTrace)
							e.printStackTrace();
					}
				}
		);

		// Create sensor subscription
		node.getNode().createSubscription(
				std_msgs.msg.String.class, sensorTopic, msg -> {
					try {
						handleSensorMessage(msg.getData());
					} catch (BeanCreationNotAllowedException ignored) {
					} catch (Exception e) {
						log.warn("Unexpected exception caught in method called by sensor subscriber");
						if (printFullStackTrace)
							e.printStackTrace();
					}
				}
		);

		// Create status subscription
		node.getNode().createSubscription(
				std_msgs.msg.String.class, statusTopic, msg -> {
					try {
						handleStatusMessage(msg.getData());
					} catch (BeanCreationNotAllowedException ignored) {
					} catch (Exception e) {
						log.warn("Unexpected exception caught in method called by status subscriber");
						if (printFullStackTrace)
							e.printStackTrace();
					}
				}
		);

		// Create mission status subscription
		node.getNode().createSubscription(
				drone_interfaces.msg.MissionStatus.class, missionTopic, msg -> {
					try {
						remoteTaskHandler.updateRemoteTaskStatus(msg);
					} catch (BeanCreationNotAllowedException ignored) {
					} catch (Exception e) {
						log.warn("Unexpected exception caught in method called by status subscriber");
						if (printFullStackTrace)
							e.printStackTrace();
					}
				}
		);

		// Create parameter events subscription
		node.getNode().createSubscription(
				ParameterEvent.class, "/parameter_events", msg -> {
					try {
						paramTracker.updateParameter(msg);
					} catch (BeanCreationNotAllowedException ignored) {
					} catch (Exception e) {
						log.warn("Unexpected exception caught in method called by status subscriber");
						if (printFullStackTrace)
							e.printStackTrace();
					}
				}
		);
	}

	private void handleTelemMessage(String msg) {
		try {
			statusUpdater.updateTelemetry(mapper.readValue(msg, Telemetry.class));
		} catch (JsonProcessingException | NotFoundException e) {
			e.printStackTrace();
		}
	}

	private void handleSensorMessage(String msg) {
		try {
			sensorMan.updateSensor(mapper.readValue(msg, HashMap.class));
		} catch (JsonProcessingException e) {
			e.printStackTrace();
		}
	}

	private void handleStatusMessage(String msg) {
		StatusMessage statusMsg;
		try {
			statusMsg = mapper.readValue(msg, StatusMessage.class);
		} catch (JsonProcessingException e) {
			e.printStackTrace();
			return;
		}
		if (statusMsg.getDroneId() == null) {
			log.warn("Received malformed status command: missing droneId");
			return;
		}
		if (statusMsg.getState() == null) {
			log.info("Received malformed status command from drone " + statusMsg.getDroneId() + ": missing state");
			return;
		}

		try {
			switch (statusMsg.getState()) {
				case connect:
				case disconnect:
					statusUpdater.setDroneSysConnection(statusMsg);
					break;
				case disarm:
				case queued:
				case stop_manual:
				case stop_offboard:
					statusUpdater.reportStatelessCommand(statusMsg);
					break;
				case start_manual:
					statusUpdater.reportStartManual(statusMsg);
					break;
				case return_to_launch:
					statusMsg.setCommand(CommandEnum.return_to_launch);
				case start:
					statusUpdater.startDroneCommand(statusMsg);
					break;
				case stop:
				case cancel:
				case finish:
					statusUpdater.stopDroneCommand(statusMsg);
					break;
				case success:
					statusUpdater.reportSuccessfulCommand(statusMsg);
					break;
				case failure:
					statusUpdater.reportFailedCommand(statusMsg);
					break;
			}
		} catch (NotFoundException e) {
			log.error("Received status command from unregistered drone");
		}
	}
}
