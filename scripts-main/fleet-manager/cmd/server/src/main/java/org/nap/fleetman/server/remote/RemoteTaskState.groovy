package org.nap.fleetman.server.remote

// State of remote task
enum RemoteTaskState {
	stopped, paused, running, emergency

	String toString() {
		return name().capitalize()
	}
}
