package org.nap.fleetman.server.exceptions

import org.nap.fleetman.server.model.mission.MissionEndMsg

class DroneAssignmentException extends MissionException {
    enum State {REPLACE, REQUIREMENT, UNAVAILABLE, NOT_REGISTERED}

    DroneAssignmentException(State state, String arg=null) {
        super(MissionEndMsg.REQUIREMENTS, createMessage(state, arg))
    }

    private static String createMessage(State state, String arg=null) {
        switch(state){
            case State.REPLACE:
                "No drones available to replace another drone"
                break
            case State.REQUIREMENT:
                "No drone available for '$arg' requirement"
                break
            case State.UNAVAILABLE:
                "Could not assign drone with id \"$arg\": unavailable"
                break
            case State.NOT_REGISTERED:
                "Could not assign drone with id \"$arg\": not registered"
                break
        }
    }
}
