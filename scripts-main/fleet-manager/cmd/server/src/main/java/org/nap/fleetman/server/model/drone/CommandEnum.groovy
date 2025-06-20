package org.nap.fleetman.server.model.drone

import com.fasterxml.jackson.annotation.JsonCreator
import com.fasterxml.jackson.annotation.JsonValue

enum CommandEnum {
    arm("arm"),
    disarm("disarm"),
    takeoff("takeoff"),
    land("land"),
    go_to("goto"),
    return_to_launch("return"),
    start_offboard("start_offboard"),
    stop_offboard("stop_offboard"),
    position_ned("position_ned"),
    velocity_ned("velocity_ned"),
    velocity_body("velocity_body"),
    attitude("attitude"),
    attitude_rate("attitude_rate"),
    actuator_control("actuator_control"),
    cancel("cancel"),
    turn("turn"),
    move("move"),
    unknown("unknown")

    private String value

    CommandEnum(String value) {
        this.value = value
    }

    @Override
    @JsonValue
    String toString() {
        value
    }

    String toPrettyString() {
        if (this == go_to)
            return "go to"
        if (this == return_to_launch)
            return "return to launch"
        if (value.contains("_"))
            return value.replace("_", " ")
        return value
    }

    @JsonCreator
    static CommandEnum fromValue(String text) {
        for (CommandEnum b : values()) {
            if (String.valueOf(b.value) == text) {
                return b
            }
        }
        return unknown
    }
}