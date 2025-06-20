package org.nap.fleetman.server.model.mission;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;

/**
 * Mission's running status
 */
public enum MissionResumeAction {
	LAST_POSITION("last_position"),
	COMPLETE_CURRENT("complete_current"),
	START_NEXT("start_next");

	private String value;

	MissionResumeAction(String value) {
		this.value = value;
	}

	@Override
	@JsonValue
	public String toString() {
		return String.valueOf(value);
	}

	@JsonCreator
	public static MissionResumeAction fromValue(String text) {
		for (MissionResumeAction b : MissionResumeAction.values()) {
			if (String.valueOf(b.value).equals(text)) {
				return b;
			}
		}
		return null;
	}

	public static MissionResumeAction getDefault() {
		return MissionResumeAction.LAST_POSITION;
	}
}
