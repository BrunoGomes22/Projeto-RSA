package org.nap.fleetman.server.model.drone

import org.nap.fleetman.server.model.telemetry.Position
import org.nap.fleetman.server.model.telemetry.RelativePosition

class StatusMessage {
    long timestamp
    String droneId
    StateMsgEnum state
    String error
    CommandEnum command
    Position coords
    RelativePosition pos
    float altitude
    float degrees


    @Override
    String toString() {
        def status = state.toPrettyString().capitalize()
        if (state == StateMsgEnum.failure)
            status += " on ${command.toPrettyString()}: $error"
        else if (state == StateMsgEnum.success)
            status += " on ${command.toPrettyString()}"
        else if (command != null && state != StateMsgEnum.return_to_launch)
            status += " ${commandToString()}"
        status
    }

    String commandToString() {
        def cmd = command.toPrettyString()
        if (command == CommandEnum.go_to)
            cmd += " ${coords.lat.trunc(6)}, ${coords.lon.trunc(6)} at ${coords.alt.trunc(2)}m"
        else if (command == CommandEnum.move) {
            if (pos.forward > 0)
                cmd += " forward ${pos.forward}m"
            else if (pos.forward < 0)
                cmd += " backward ${-pos.forward}m"
            if (pos.right > 0)
                cmd += " right ${pos.right}m"
            else if (pos.right < 0)
                cmd += " left ${-pos.right}m"
            if (pos.up > 0)
                cmd += " up ${pos.up}m"
            else if (pos.up < 0)
                cmd += " down ${-pos.up}m"
            cmd += " (${coords.lat.trunc(6)}, ${coords.lon.trunc(6)} at ${coords.alt.trunc(2)}m)"
        }else if (state == StateMsgEnum.start) {
            if (command == CommandEnum.takeoff || command == CommandEnum.return_to_launch)
                cmd += " with altitude ${altitude}m"
            else if (command == CommandEnum.turn)
                cmd += " by ${degrees.trunc(2)} degrees"
        }
        cmd
    }
}
