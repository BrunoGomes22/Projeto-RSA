package org.nap.fleetman.server.model.drone;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;

/**
 * Drone state error
 */
public enum Error {
	BATTERY_CRIT("Critical battery level"),
	HEALTH_FAIL("Failed health check");

	private final String value;

	Error(String value) {
		this.value = value;
	}

	@Override
	@JsonValue
	public String toString() {
		return String.valueOf(value);
	}

	@JsonCreator
	public static Error fromValue(String text) {
		for (Error b : Error.values()) {
			if (String.valueOf(b.value).equals(text)) {
				return b;
			}
		}
		return null;
	}
}
