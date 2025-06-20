package org.nap.fleetman.server.model.mission;
import org.nap.fleetman.server.model.dsl.DroneWrapper;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import org.nap.fleetman.server.api.TimestampSerializer;
import org.nap.fleetman.server.remote.RemoteTaskDTO;
import org.springframework.validation.annotation.Validated;

import javax.validation.Valid;
import java.util.*;

/**
 * Mission as retrieved from the system.
 */
@Validated
public class Mission {
	@JsonProperty("missionId")
	private String missionId = null;

	@JsonProperty("status")
	private MissionStatus status = null;

	@JsonInclude(JsonInclude.Include.NON_NULL)
	@JsonProperty("cause")
	private MissionEndMsg cause = null;

	@JsonProperty("start")
	@JsonSerialize(using = TimestampSerializer.class)
	private Long start = null;

	@JsonProperty("end")
	@JsonSerialize(using = TimestampSerializer.class)
	private Long end = null;

	@JsonProperty("plugins")
	private Set<String> plugins = null;

	@JsonProperty("remoteTasks")
	private List<RemoteTaskDTO> remoteTasks;

	@JsonProperty("activeDrones")
	@Valid
	private Set<String> activeDrones = null;

	@JsonProperty("usedDrones")
	@Valid
	private Set<String> usedDrones = null;
	
	@JsonProperty("pausedDrones")
	@Valid
	private Map<String, DroneWrapper> pausedDrones = null;

	private Map<String, Set<String>> requiredDrones = null;


	public Mission() {
		plugins = new HashSet<>();
		activeDrones = new HashSet<>();
		usedDrones = new HashSet<>();
		pausedDrones = new HashMap<>();
		requiredDrones = new HashMap<>();
	}

	public Mission(String missionId) {
		this.missionId = missionId;
		plugins = new HashSet<>();
		activeDrones = new HashSet<>();
		usedDrones = new HashSet<>();
		pausedDrones = new HashMap<>();
		requiredDrones = new HashMap<>();
		remoteTasks = new ArrayList<>();
	}

	public String getMissionId() {
		return missionId;
	}

	public void setMissionId(String missionId) {
		this.missionId = missionId;
	}

	public MissionStatus getStatus() {
		return status;
	}

	public void setStatus(MissionStatus status) {
		this.status = status;
	}

	public MissionEndMsg getCause() {
		return cause;
	}

	public void setCause(MissionEndMsg cause) {
		this.cause = cause;
	}

	public Long getStart() {
		return start;
	}

	public void setStart(Long start) {
		this.start = start;
	}

	public Long getEnd() {
		return end;
	}

	public void setEnd(Long end) {
		this.end = end;
	}

	public Set<String> getPlugins() {
		return plugins;
	}

	public Mission addPlugin(String plugin) {
		if (this.plugins == null) {
			this.plugins = new HashSet<>();
		}
		this.plugins.add(plugin);
		return this;
	}

	public Mission removePlugin(String plugin) {
		this.plugins.remove(plugin);
		return this;
	}

	public Mission addRemoteTask(RemoteTaskDTO remoteTask) {
		if (this.remoteTasks == null)
			this.remoteTasks = new ArrayList<>();
		this.remoteTasks.add(remoteTask);
		return this;
	}

	public List<RemoteTaskDTO> getRemoteTasks() {
		return remoteTasks;
	}

	public Set<String> getActiveDrones() {
		return activeDrones;
	}

	public Mission addActiveDrone(String drone) {
		if (this.activeDrones == null) {
			this.activeDrones = new HashSet<>();
		}
		this.activeDrones.add(drone);
		return this;
	}

	public Mission removeActiveDrone(String drone) {
		this.activeDrones.remove(drone);
		return this;
	}

	public Mission clearActiveDrones() {
		this.activeDrones.clear();
		return this;
	}

	public Set<String> getUsedDrones() {
		return usedDrones;
	}

	public Mission addUsedDrone(String drone) {
		if (this.usedDrones == null) {
			this.usedDrones = new HashSet<>();
		}
		this.usedDrones.add(drone);
		return this;
	}

	@JsonIgnore
	public Map<String, DroneWrapper> getPausedDrones() {
		return pausedDrones;
	}

	public Mission addPausedDrone(String droneId, DroneWrapper drone) {
		if (this.pausedDrones == null) {
			this.pausedDrones = new HashMap<>();
		}
		this.pausedDrones.put(droneId, drone);
		return this;
	}

	public Mission removePausedDrone(String droneId) {
		this.pausedDrones.remove(droneId);
		return this;
	}

	@JsonIgnore
	public Map<String, Set<String>> getRequiredDrones() {
		return requiredDrones;
	}

	public Mission addRequiredDrone(String drone, Set<String> sensors) {
		if (this.requiredDrones == null) {
			this.requiredDrones = new HashMap<>();
		}
		this.requiredDrones.put(drone, sensors);
		return this;
	}

	public Mission removeRequiredDrone(String drone) {
		this.requiredDrones.remove(drone);
		return this;
	}

	@Override
	public boolean equals(java.lang.Object o) {
		if (this == o) {
			return true;
		}
		if (o == null || getClass() != o.getClass()) {
			return false;
		}
		Mission mission = (Mission) o;
		return Objects.equals(this.missionId, mission.missionId);
	}

	@Override
	public int hashCode() {
		return Objects.hash(missionId);
	}
}
