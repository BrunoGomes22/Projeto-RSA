package org.nap.fleetman.server.core;

import org.nap.fleetman.server.api.NotFoundException;
import org.nap.fleetman.server.mission.MissionManager;
import org.nap.fleetman.server.concurrent.SyncChannel;
import org.nap.fleetman.server.mission.WrapperManager;
import org.nap.fleetman.server.model.drone.Error;
import org.nap.fleetman.server.model.drone.*;
import org.nap.fleetman.server.model.mission.MissionEndMsg;
import org.nap.fleetman.server.model.telemetry.LandState;
import org.nap.fleetman.server.model.telemetry.Telemetry;
import org.nap.fleetman.server.utils.CommandHandler;
import org.nap.fleetman.server.utils.DroneLog;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

/**
 * Process drone telemetry and status updates
 */
@Service
public class StatusUpdater {
	private final DroneManager droneManager;
	private final MissionManager missionManager;
	private final WrapperManager wrapperMan;
	private final SyncChannel syncChannel;
	private final CommandHandler cmdHandler;
	@Value("${fleetman.drone.batteryLevel.critical:0.05}")
	private double criticalBatteryLevel;
	@Value("${fleetman.drone.batteryLevel.low:0.15}")
	private double lowBatteryLevel;
	@Value("${fleetman.mission.error.battery.return:true}")
	private boolean returnOnCriticalBattery;

	StatusUpdater(DroneManager droneManager, MissionManager missionManager, WrapperManager wrapperMan, SyncChannel syncChannel, CommandHandler cmdHandler) {
		this.droneManager = droneManager;
		this.missionManager = missionManager;
		this.wrapperMan = wrapperMan;
		this.syncChannel = syncChannel;
		this.cmdHandler = cmdHandler;
	}

	// Update telemetry data to the latest
	public void updateTelemetry(Telemetry telem) throws NotFoundException {
		// Create drone if it doesn't exist
		getOrCreateDrone(telem.getDroneId());
		// Update telemetry
		droneManager.updateTelemetry(telem);
		// Update drone state
		updateDroneState(telem);
	}

	// Retrieve drone from manager or create it if it doesn't exist
	public Drone getOrCreateDrone(String droneId) {
		Drone drone;
		try {
			drone = droneManager.getDrone(droneId);
		} catch (NotFoundException e) {
			drone = droneManager.createDrone(droneId);
			DroneLog.info(droneId, "Registered");
			droneManager.updateTelemetry(new Telemetry(droneId));
		}
		return drone;
	}

	// Update drone's command
	public void startDroneCommand(StatusMessage msg) throws NotFoundException {
		Drone drone = getOrCreateDrone(msg.getDroneId());
		// Set the command
		drone.setCurrentCommand(new Command(msg));
		// Log the received command
		DroneLog.info(msg);
		// Update drone state
		updateDroneState(drone);
		// If the status message regarded a system-triggered return to launch, cancel mission
		if (drone.getMission() != null && msg.getState() == StateMsgEnum.return_to_launch)
				missionManager.cancelMission(drone.getMission(), MissionEndMsg.RETURN);
	}

	// Remove drone's current command
	public void stopDroneCommand(StatusMessage msg) throws NotFoundException {
		Drone drone = getOrCreateDrone(msg.getDroneId());
		drone.setCurrentCommand(null);
		// Log the received command
		DroneLog.info(msg);

		// Update drone state
		updateDroneState(drone);

		if (drone.getMission() != null && drone.getRemoteTaskType() == null)
			syncChannel.unlockCommand(msg.getDroneId(), msg.getCommand().toString());
	}

	// Add or remove warn regarding Flight Controller's connectivity
	public void setDroneSysConnection(StatusMessage msg) throws NotFoundException {
		Drone drone = getOrCreateDrone(msg.getDroneId());
		if (msg.getState() == StateMsgEnum.connect) {
			drone.removeWarn(Warn.DISCONNECT);
			DroneLog.info(drone.getDroneId(), "Flight controller system connected", msg.getTimestamp());
		} else {
			drone.addWarn(Warn.DISCONNECT);
			DroneLog.warn(drone.getDroneId(), Warn.DISCONNECT.toString(), msg.getTimestamp());
		}
		updateDroneState(drone);
	}

	// Log received status message that doesn't alter drone's state
	public void reportStatelessCommand(StatusMessage msg) {
		DroneLog.info(msg);
	}

	// Log failed command
	public void reportFailedCommand(StatusMessage msg) throws NotFoundException {
		DroneLog.error(msg);
		Drone drone = droneManager.getDrone(msg.getDroneId());
		if (drone.getMission() != null && drone.getRemoteTaskType() == null) {
			missionManager.concludeFailedMission(drone.getMission(), MissionEndMsg.CMD_EXEC);
			syncChannel.unlockCommand(msg.getDroneId(), msg.getCommand().toString());
		}
	}

	// Log successful command
	public void reportSuccessfulCommand(StatusMessage msg) throws NotFoundException {
		DroneLog.info(msg);
		Drone drone = droneManager.getDrone(msg.getDroneId());
		if (drone.getMission() != null && drone.getRemoteTaskType() == null)
			syncChannel.unlockCommand(msg.getDroneId(), msg.getCommand().toString());
	}

	// Log starting manual mode
	public void reportStartManual(StatusMessage msg) throws NotFoundException {
		DroneLog.info(msg);
		Drone drone = droneManager.getDrone(msg.getDroneId());
		if (drone.getMission() != null)
			missionManager.cancelMission(drone.getMission(), MissionEndMsg.MANUAL);
	}

	private void updateDroneState(Drone drone) throws NotFoundException {
		updateDroneState(drone, droneManager.getTelemetry(drone.getDroneId()));
	}

	private void updateDroneState(Telemetry telem) throws NotFoundException {
		updateDroneState(droneManager.getDrone(telem.getDroneId()), telem);
	}

	// Update drone's current state and state description
	private void updateDroneState(Drone drone, Telemetry telem) throws NotFoundException {
		// Hasn't received telemetry message yet
		if (telem.getTimestamp() == null)
			return;

		boolean hadNoErrors = drone.getErrors().isEmpty();
		// Update errors and warnings
		boolean triggerReturn = updateDroneErrorsAndWarns(drone, telem);

		if (drone.getState() == DroneState.UNKNOWN)
			DroneLog.info(drone.getDroneId(), "Established connection with ground station");


		//DroneLog.info(drone.getDroneId(), "Drone states:");
		//DroneLog.info(drone.getDroneId(), (telem.getLandState() == LandState.IN_AIR) ? "true" : "false");
		//DroneLog.info(drone.getDroneId(), (drone.getCurrentCommand() == null) ? "true" : "false");
		//DroneLog.info(drone.getDroneId(), (drone.getMission() == null) ? "true" : "false");

		// If drone has pending errors
		if (!drone.getErrors().isEmpty()) {
			drone.setState(DroneState.ERROR);
			// If drone is on ground
		} else if (telem.getLandState() == LandState.ON_GROUND){
			drone.setState(DroneState.READY);
			// If drone is in manual mode
		} else if (telem.getFlightMode().isManual()) {
			drone.setState(DroneState.MANUAL);
			// If drone is running a command
		} else if (drone.getCurrentCommand() != null) {
			drone.setState(DroneState.ACTIVE);
			// If drone is in the air but without command
		} else if (telem.getLandState() == LandState.IN_AIR && drone.getCurrentCommand() == null && drone.getMission() == null) {
			//DroneLog.info(drone.getDroneId(), "Drone is ready in air");
			drone.setState(DroneState.READYINAIR);
			// If drone is on hold
		} else {
			//DroneLog.info(drone.getDroneId(), "Drone is on hold");
			drone.setState(DroneState.HOLD);
		}

		// Notify mission manager that this drone encountered an error
		if (drone.getMission() != null && (!drone.getErrors().isEmpty() && hadNoErrors)) {
			missionManager.concludeFailedMission(drone.getMission(), MissionEndMsg.DRONE_ERROR);
			if (triggerReturn && returnOnCriticalBattery) {
				CommandEnum cmd = drone.getCurrentCommand().getType();
				if (cmd != CommandEnum.land && cmd != CommandEnum.return_to_launch)
					cmdHandler.returnToLaunch(drone.getDroneId(), null, null);
			}
		}

		// Persist updates
		droneManager.updateDrone(drone);
		if (drone.getMission() != null)
			wrapperMan.updateWrapperTelem(drone, telem);
	}

	// Check if new warns/errors have risen or been solved
	private boolean updateDroneErrorsAndWarns(Drone drone, Telemetry telem) {
		boolean triggerReturn = false;
		// Check battery
		if (telem.getBattery().getPercentage() < criticalBatteryLevel) {
			if (!drone.getErrors().contains(Error.BATTERY_CRIT)) {
				drone.addError(Error.BATTERY_CRIT);
				DroneLog.error(drone.getDroneId(), Error.BATTERY_CRIT.toString());
				triggerReturn = true;
			}
			if (drone.getWarns().contains(Warn.BATTERY_LOW))
				drone.removeWarn(Warn.BATTERY_LOW);
		} else if (drone.getErrors().contains(Error.BATTERY_CRIT)) {
			drone.removeError(Error.BATTERY_CRIT);
		} else if (telem.getBattery().getPercentage() < lowBatteryLevel) {
			if (!drone.getWarns().contains(Warn.BATTERY_LOW)) {
				drone.addWarn(Warn.BATTERY_LOW);
				DroneLog.warn(drone.getDroneId(), Warn.BATTERY_LOW.toString());
			}
		} else if (drone.getWarns().contains(Warn.BATTERY_LOW)) {
			drone.removeWarn(Warn.BATTERY_LOW);
		}

		// Check health failures
		if (!telem.getHealthFailures().isEmpty() && !drone.getErrors().contains(Error.HEALTH_FAIL)) {
			drone.addError(Error.HEALTH_FAIL);
			DroneLog.error(drone.getDroneId(), Error.HEALTH_FAIL.toString());
		} else if (telem.getHealthFailures().isEmpty() && drone.getErrors().contains(Error.HEALTH_FAIL))
			drone.removeError(Error.HEALTH_FAIL);
		return triggerReturn;
	}
}
