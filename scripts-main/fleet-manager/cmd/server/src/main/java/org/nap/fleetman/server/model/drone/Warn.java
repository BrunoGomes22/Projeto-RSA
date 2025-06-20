package org.nap.fleetman.server.model.drone;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;

/**
 * Drone state warn
 */
public enum Warn {
	BATTERY_LOW("Low battery level"),
	DISCONNECT("Flight controller system disconnected");

	private final String value;

	Warn(String value) {
		this.value = value;
	}

	@Override
	@JsonValue
	public String toString() {
		return String.valueOf(value);
	}

	@JsonCreator
	public static Warn fromValue(String text) {
		for (Warn b : Warn.values()) {
			if (String.valueOf(b.value).equals(text)) {
				return b;
			}
		}
		return null;
	}
}
