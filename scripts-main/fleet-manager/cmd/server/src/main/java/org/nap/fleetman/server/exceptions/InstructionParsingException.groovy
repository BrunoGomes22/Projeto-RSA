package org.nap.fleetman.server.exceptions

import org.nap.fleetman.server.model.mission.MissionEndMsg

class InstructionParsingException extends MissionException {
    enum State {NOT_COORDINATES, EMPTY_SENSORS, UNRECOGNIZED_SENSORS, UNRECOGNIZED_REQ,
                INVALID_PARAM, INVALID_MOVE, INVALID_DIR, UNRECOGNIZED_PARAM}

    InstructionParsingException(State state, arg=null) {
        super(state.ordinal() < 4 ? MissionEndMsg.REQ_PARSE : MissionEndMsg.CMD_PARSE, createMessage(state, arg))
    }

    private static String createMessage(State state, arg=null) {
        switch (state) {
            case State.NOT_COORDINATES:
                "Provided assignment requirement map does not represent a coordinate"
                break
            case State.EMPTY_SENSORS:
                "Provided empty sensor list as assignment requirement"
                break
            case State.UNRECOGNIZED_SENSORS:
                "Provided required sensor list with unrecognized token '$arg'"
                break
            case State.UNRECOGNIZED_REQ:
                "Unrecognized drone assignment requirement '$arg'"
                break
            case State.INVALID_PARAM:
                "Invalid value for command parameter '${arg.param}': ${arg.value}"
                break
            case State.INVALID_MOVE:
                "Invalid move command: $arg"
                break
            case State.INVALID_DIR:
                "Invalid direction: $arg"
                break
            case State.UNRECOGNIZED_PARAM:
                "Unrecognized parameter '${arg.param}' on '${arg.cmd}' command"
                break
        }
    }
}
