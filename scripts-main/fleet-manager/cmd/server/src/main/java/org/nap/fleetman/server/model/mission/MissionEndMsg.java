package org.nap.fleetman.server.model.mission;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonValue;

/**
 * Mission conclusion status message
 */
public enum MissionEndMsg {
	SUCCESS("success"),
	REQUIREMENTS("not enough drones to fulfill mission requirements"),
	REQ_PARSE("failure in assignment requirement parsing"),
	CMD_EXEC("failure in command execution"),
	CMD_PARSE("failure in command parsing"),
	PLUGIN_ERROR("plugin encountered an error"),
	PLUGIN_PARAM("illegal or missing plugin input parameter"),
	PLUGIN_MISSING("plugin is not loaded"),
	ILLEGAL_TASK("illegal task handling request"),
	IRREPLACEABLE("requested replacement of drone that can not be replaced"),
	PARAM_FAIL("unable to get or set parameter"),
	PARAM_NODE("unable handle parameter of nonexistent node"),
	MANUAL("manual mode activated"),
	RETURN("automatic return to launch detected"),
	USER("user request"),
	TIMEOUT("drone timeout"),
	SENSOR_TIMEOUT("sensor timeout"),
	SENSOR_UNKNOWN("unknown sensor"),
	SCRIPT("malformed script"),
	REVOKED("referenced revoked drone"),
	DRONE_ERROR("drone entered error state"),
	REMOTE_TASK("failure while handling remote task"),
	SECURITY("mission script called a blacklisted method or import"),
	UNEXPECTED("unexpected exception");

	private String value;

	MissionEndMsg(String value) {
		this.value = value;
	}

	@Override
	@JsonValue
	public String toString() {
		return String.valueOf(value);
	}

	@JsonCreator
	public static MissionEndMsg fromValue(String text) {
		for (MissionEndMsg b : MissionEndMsg.values()) {
			if (String.valueOf(b.value).equals(text)) {
				return b;
			}
		}
		return null;
	}
}
