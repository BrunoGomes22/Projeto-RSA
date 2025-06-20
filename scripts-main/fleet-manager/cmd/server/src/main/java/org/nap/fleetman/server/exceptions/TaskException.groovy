package org.nap.fleetman.server.exceptions

import org.nap.fleetman.server.model.mission.MissionEndMsg

class TaskException extends MissionException {
    enum State {NULL, WAIT, STOP}

    TaskException(State state) {
        super(MissionEndMsg.ILLEGAL_TASK, createMessage(state))
    }

    private static String createMessage(State state) {
        switch(state) {
            case State.NULL:
                "Request to wait for task with null reference"
                break
            case State.WAIT:
                "Request to wait for non-existent task"
                break
            case State.STOP:
                "Request to stop non-existent task"
                break
        }
    }
}
