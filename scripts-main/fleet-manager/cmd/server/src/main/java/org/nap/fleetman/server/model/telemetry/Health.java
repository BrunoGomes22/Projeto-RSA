package org.nap.fleetman.server.model.telemetry;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;

/**
 * Drone's health measures
 */
public enum Health {
	GYROMETER("gyrometer"),
	ACCELEROMETER("accelerometer"),
	MAGNETOMETER("magnetometer"),
	LEVEL("level"),
	LOCALPOS("localPos"),
	GLOBALPOS("globalPos"),
	HOMEPOS("homePos");

	private String value;

	Health(String value) {
		this.value = value;
	}

	@Override
	@JsonValue
	public String toString() {
		return String.valueOf(value);
	}

	@JsonCreator
	public static Health fromValue(String text) {
		for (Health b : Health.values()) {
			if (String.valueOf(b.value).equals(text)) {
				return b;
			}
		}
		return null;
	}
}
