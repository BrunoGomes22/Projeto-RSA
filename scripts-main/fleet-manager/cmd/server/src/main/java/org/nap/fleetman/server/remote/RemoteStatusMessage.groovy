package org.nap.fleetman.server.remote

// Status message types sent by remote services
enum RemoteStatusMessage {
	progress, state, error, info,
	// FRIENDS-specific message types
	planning, following
}
