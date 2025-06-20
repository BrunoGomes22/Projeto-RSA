package org.nap.fleetman.server.core;

import org.nap.fleetman.server.api.NotFoundException;
import org.nap.fleetman.server.mission.MissionManager;
import org.nap.fleetman.server.model.drone.Drone;
import org.nap.fleetman.server.model.drone.DroneState;
import org.nap.fleetman.server.model.mission.MissionEndMsg;
import org.nap.fleetman.server.model.telemetry.Telemetry;
import org.nap.fleetman.server.sensors.SensorData;
import org.nap.fleetman.server.sensors.SensorManager;
import org.nap.fleetman.server.utils.DroneLog;
import org.nap.fleetman.server.utils.MissionUtils;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Service;

import java.time.Duration;
import java.time.Instant;
import java.util.List;

@Service("FleetmanTaskScheduler")
public class TaskScheduler {
	private final DroneManager droneMan;
	private final MissionManager missionMan;
	private final SensorManager sensorMan;
	private final MessagePublisher messagePub;
	@Value("${fleetman.rate.droneTimeout:5000}")
	private long droneTimeout;

	public TaskScheduler(DroneManager droneMan, MissionManager missionMan, SensorManager sensorMan, MessagePublisher messagePub) {
		this.droneMan = droneMan;
		this.missionMan = missionMan;
		this.sensorMan = sensorMan;
		this.messagePub = messagePub;
	}

	private long timeElapsed(long timestamp) {
		return Duration.between(Instant.ofEpochMilli(timestamp), Instant.now()).toMillis();
	}

	// FIXME more efficient timeout verification, such as what is done for sensors
	// Monitors if the ground station has stopped receiving drone telemetry messages
	@Scheduled(fixedDelayString = "${fleetman.rate.droneTimeout:5000}")
	private void monitorDroneTimeout() {
		List<Telemetry> telems = droneMan.getTelemetryByTimestamp();
		for (Telemetry telem : telems) {
			if (telem.getTimestamp() == null)
				continue;
			try {
				long timeElapsed = timeElapsed(telem.getTimestamp());
				Drone drone = droneMan.getDrone(telem.getDroneId());
				if (drone.getState() != DroneState.UNKNOWN && timeElapsed > droneTimeout) {
					DroneLog.warn(drone.getDroneId(), "Stopped receiving messages from drone");
					droneMan.setDroneTimeout(drone);
					if (drone.getMission() != null)
						missionMan.concludeFailedMission(drone.getMission(), MissionEndMsg.TIMEOUT);
				} else return;
			} catch (NotFoundException e) {
				e.printStackTrace();
			}
		}
	}

	// Monitors if the ground station has stopped receiving sensor messages
	@Scheduled(fixedDelayString = "${fleetman.rate.sensorTimeout:5000}")
	private void monitorSensorTimeout() {
		for (SensorData sensor : sensorMan.getTimedOutSensors()) {
			if (!sensorMan.removeSensor(sensor))
				break;
			// If sensor is attached to drone
			if (sensor.getDroneId() != null && missionMan.isDroneInMission(sensor.getDroneId())) {
				String missionId = missionMan.getDroneMissionId(sensor.getDroneId());
				if (missionMan.isDroneRequired(missionId, sensor.getDroneId())
						&& missionMan.getDroneMissionRequirements(missionId, sensor.getDroneId()).contains(sensor.getType())) {
					try {
						missionMan.concludeFailedMission(missionId, MissionEndMsg.SENSOR_TIMEOUT);
					} catch (NotFoundException ignored) {
					}
				}
			}
		}
	}

	// Sends a periodic heartbeat message with the current timestamp
	@Scheduled(fixedRateString = "${fleetman.rate.heartbeat:1000}")
	private void sendHeartbeat() {
		messagePub.publishHeartbeat(Instant.now().toEpochMilli());
	}
}
