package org.nap.fleetman.server.model.telemetry;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;

/**
 * Drone's landing state
 */
public enum LandState {
	UNKNOWN("unknown"),
	ON_GROUND("on_ground"),
	IN_AIR("in_air"),
	TAKING_OFF("taking_off"),
	LANDING("landing");

	private String value;

	LandState(String value) {
		this.value = value;
	}

	@Override
	@JsonValue
	public String toString() {
		return String.valueOf(value);
	}

	@JsonCreator
	public static LandState fromValue(String text) {
		for (LandState b : LandState.values()) {
			if (String.valueOf(b.value).equals(text)) {
				return b;
			}
		}
		return null;
	}
}
