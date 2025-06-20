package org.nap.fleetman.server.model.telemetry;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;

/**
 * Drone's active flight mode
 */
public enum FlightMode {
	UNKNOWN("unknown"),
	READY("ready"),
	TAKEOFF("takeoff"),
	HOLD("hold"),
	MISSION("mission"),
	RETURN_TO_LAUNCH("return_to_launch"),
	LAND("land"),
	OFFBOARD("offboard"),
	FOLLOW_ME("follow_me"),
	MANUAL("manual"),
	ALTCTL("altctl"),
	POSCTL("posctl"),
	ACRO("acro"),
	STABILIZED("stabilized"),
	RATTITUDE("rattitude");

	private String value;

	FlightMode(String value) {
		this.value = value;
	}

	@Override
	@JsonValue
	public String toString() {
		return String.valueOf(value);
	}

	@JsonCreator
	public static FlightMode fromValue(String text) {
		for (FlightMode b : FlightMode.values()) {
			if (String.valueOf(b.value).equals(text)) {
				return b;
			}
		}
		return null;
	}

	public boolean isManual() {
		return this.compareTo(OFFBOARD) > 0;
	}
}