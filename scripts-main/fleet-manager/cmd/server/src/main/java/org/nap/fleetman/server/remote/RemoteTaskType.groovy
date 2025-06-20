package org.nap.fleetman.server.remote

import com.fasterxml.jackson.annotation.JsonValue

// Tasks that are performed by remote instances
enum RemoteTaskType {
	monitoring,	scouting, inspection, pathfollower

	@JsonValue
	String toString() {
		return name().toLowerCase()
	}
}
