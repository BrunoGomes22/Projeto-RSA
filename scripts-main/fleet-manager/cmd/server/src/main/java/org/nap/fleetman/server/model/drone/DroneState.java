package org.nap.fleetman.server.model.drone;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;

/**
 * Drone's current state
 */
public enum DroneState {
	UNKNOWN("unknown"),
	ERROR("error"),
	MANUAL("manual"),
	ACTIVE("active"),
	HOLD("hold"),
	READY("ready"),
	READYINAIR("readyinair");

	private final String value;

	DroneState(String value) {
		this.value = value;
	}

	@Override
	@JsonValue
	public String toString() {
		return String.valueOf(value);
	}

	@JsonCreator
	public static DroneState fromValue(String text) {
		for (DroneState b : DroneState.values()) {
			if (String.valueOf(b.value).equals(text)) {
				return b;
			}
		}
		return null;
	}
}
