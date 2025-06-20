package org.nap.fleetman.server.core;

import groovyx.gpars.dataflow.Dataflows;
import org.nap.fleetman.server.api.NotFoundException;
import org.nap.fleetman.server.model.drone.Drone;
import org.nap.fleetman.server.model.drone.DroneState;
import org.nap.fleetman.server.model.telemetry.FlightMode;
import org.nap.fleetman.server.model.telemetry.Telemetry;
import org.nap.fleetman.server.parameters.ParameterTracker;
import org.nap.fleetman.server.remote.RemoteTaskType;
import org.nap.fleetman.server.repo.DroneRepository;
import org.nap.fleetman.server.repo.TelemRepository;
import org.nap.fleetman.server.utils.DroneLog;
import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.stream.Collectors;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Service for drone management, in which it is possible to retrieve and update persistent information.
 */
@Service
public class DroneManager {
	private static final Logger log = LoggerFactory.getLogger(DroneManager.class);

	private final ParameterTracker paramTracker;
	private final DroneRepository droneRep;
	private final TelemRepository telemRep;
	private final ConcurrentHashMap<String, String> pendingSensors;

	DroneManager(ParameterTracker paramTracker, DroneRepository droneRep, TelemRepository telemRep) {
		this.paramTracker = paramTracker;
		this.droneRep = droneRep;
		this.telemRep = telemRep;
		pendingSensors = new ConcurrentHashMap<>();
	}

	public boolean droneExists(String droneId) {
		return droneRep.findByDroneId(droneId) != null;
	}

	// Get a drone from repo
	public Drone getDrone(String droneId) throws NotFoundException {
		Drone drone = droneRep.findByDroneId(droneId);
		if (drone == null)
			throw new NotFoundException("Drone id not found.");
		return drone;
	}

	// Get multiple drones from repo
	public List<Drone> getDrones(DroneState state, List<String> sensorTypes) {
		List<Drone> drones;
		if (state == null)
			drones = droneRep.findAll();
		else
			drones = droneRep.findByState(state);

		// FIXME this isn't nice but was chosen due to the inability to describe it with a findBy query; should be
		//  replaced by an actual query
		if (sensorTypes != null)
			return filterDronesBySensorTypes(drones, sensorTypes);
		return drones;
	}

	private List<Drone> filterDronesBySensorTypes(List<Drone> drones, List<String> sensorTypes) {
		return drones.stream().filter(d -> d.getSensors().containsAll(sensorTypes)).collect(Collectors.toList());
	}

	public List<Drone> getAvailableDrones(List<String> sensorTypes) {
		List<Drone> availableDrones = new ArrayList<>();
		List<Drone> drones = droneRep.findByStateAndMission(DroneState.READY, null);
		drones.addAll(droneRep.findByStateAndMission(DroneState.READYINAIR, null));
		if (sensorTypes != null)
			drones = filterDronesBySensorTypes(drones, sensorTypes);
		for (Drone drone : drones) {
			if ((Boolean) paramTracker.getParameter(drone.getDroneId(), "assignable"))
				availableDrones.add(drone);
		}
		return availableDrones;
	}

	// Get drone's telemetry from repo
	public Telemetry getTelemetry(String droneId) throws NotFoundException {
		Telemetry telem = telemRep.findByDroneId(droneId);
		if (telem == null)
			throw new NotFoundException("Drone id not found.");
		return telem;
	}

	// Create new drone
	public Drone createDrone(String droneId) {
		Drone drone = new Drone(droneId);
		// Add sensor to the drone's available sensor list if a sensor message was received before the drone registered
		if (pendingSensors.containsKey(drone.getDroneId()))
			drone.addSensor(pendingSensors.remove(drone.getDroneId()));
		updateDrone(drone);
		return drone;
	}

	// Set by the TaskScheduler when the groundstation stops receiving telemetry messages from this drone
	public void setDroneTimeout(Drone drone) {
		drone.setState(DroneState.UNKNOWN);
		drone.setCurrentCommand(null);
		updateDrone(drone);
	}

	// Set when the drone starts running a task that is executed remotely without groundstation control
	public void setDroneRemoteTask(String droneId, RemoteTaskType taskType) throws NotFoundException {
		Drone drone = getDrone(droneId);
		drone.setRemoteTaskType(taskType);
		updateDrone(drone);
	}

	// Get all drone's telemetry by timestamp
	public List<Telemetry> getTelemetryByTimestamp() {
		return telemRep.findAllByOrderByTimestampAsc();
	}

	// Persist drone modifications
	public void updateDrone(Drone drone) {
		droneRep.save(drone);
	}

	// Persist telemetry modifications
	public void updateTelemetry(Telemetry telemetry) {
		telemRep.save(telemetry);
	}

	// TODO might be on hold, needs to be ready
	// Assign drone to mission
	public boolean assignToMission(String droneId, String missionId) throws NotFoundException {
		Drone drone = getDrone(droneId);

		if (!canAssignDroneToMission(droneId))
			return false;

		log.info("Assigning drone " + droneId + " to mission " + missionId);

		drone.setMission(missionId);

		// Update
		updateDrone(drone);
		DroneLog.info(droneId, "Assigned to mission \"" + missionId + "\"");
		return true;
	}

	// Clear drone mission
	public void clearDroneMission(String droneId) throws NotFoundException {
		Drone drone = getDrone(droneId);
		drone.setMission(null);
		updateDrone(drone);
	}

	// Whether it is possible to assign this drone to a mission, given that it should not be already assigned to another
	// mission, it should not be running any commands and it should be marked as assignable to a mission
	public boolean canAssignDroneToMission(String droneId) throws NotFoundException {
		Drone drone = getDrone(droneId);
		return drone.getMission() == null && (drone.getState() == DroneState.READY || drone.getState() == DroneState.READYINAIR) && (Boolean) paramTracker.getParameter(droneId, "assignable");
	}

	// Whether the drone can currently execute a command
	public boolean canDroneExecuteCommand(String droneId, String missionId) throws NotFoundException {
		Drone drone = getDrone(droneId);
		Telemetry telem = getTelemetry(droneId);
		return drone.getState() != DroneState.ERROR	&& drone.getState() != DroneState.UNKNOWN && Objects.equals(missionId, drone.getMission());
	}

	public boolean addSensorToDrone(String droneId, String type) {
		Drone drone;
		try {
			drone = getDrone(droneId);
		} catch (NotFoundException e) {
			// If a sensor message referring to an unregistered drone is received, save that info until the drone is
			// registered
			pendingSensors.put(droneId, type);
			return false;
		}
		drone.addSensor(type);
		updateDrone(drone);
		return true;
	}

	public void removeSensorFromDrone(String droneId, String type) {
		Drone drone;
		try {
			drone = getDrone(droneId);
		} catch (NotFoundException e) {
			return;
		}
		drone.removeSensor(type);
		updateDrone(drone);
	}
}
