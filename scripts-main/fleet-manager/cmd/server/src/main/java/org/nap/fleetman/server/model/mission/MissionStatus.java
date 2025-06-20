package org.nap.fleetman.server.model.mission;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;

/**
 * Mission's running status
 */
public enum MissionStatus {
	PENDING("pending"),
	RUNNING("running"),
	PAUSED("paused"),
	FINISHED("finished"),
	FAILED("failed"),
	CANCELLED("cancelled");

	private String value;

	MissionStatus(String value) {
		this.value = value;
	}

	@Override
	@JsonValue
	public String toString() {
		return String.valueOf(value);
	}

	@JsonCreator
	public static MissionStatus fromValue(String text) {
		for (MissionStatus b : MissionStatus.values()) {
			if (String.valueOf(b.value).equals(text)) {
				return b;
			}
		}
		return null;
	}
}
