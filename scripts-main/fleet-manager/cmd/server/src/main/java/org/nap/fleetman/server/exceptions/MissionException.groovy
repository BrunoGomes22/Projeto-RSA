package org.nap.fleetman.server.exceptions

import org.nap.fleetman.server.model.mission.MissionEndMsg

class MissionException extends RuntimeException {
    MissionEndMsg missionEndMessage

    MissionException(MissionEndMsg missionEndMessage, String message) {
        super(message)
        this.missionEndMessage = missionEndMessage
    }
}
