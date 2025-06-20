package org.nap.fleetman.server.model.drone

import com.fasterxml.jackson.annotation.JsonCreator
import com.fasterxml.jackson.annotation.JsonValue

enum StateMsgEnum {
    start("start"),
    stop("stop"),
    cancel("cancel"),
    finish("finish"),
    queued("queued"),
    success("success"),
    failure("failure"),
    connect("connect"),
    disconnect("disconnect"),
    disarm("disarm"),
    return_to_launch("return"),
    stop_offboard("stop_offboard"),
    start_manual("start_manual"),
    stop_manual("stop_manual"),
    unknown("unknown")

    private String value
    StateMsgEnum(String value) {
        this.value = value
    }

    @Override
    @JsonValue
    String toString() {
        String.valueOf(value)
    }

    String toPrettyString() {
        if (this == disarm)
            return "auto disarmed"
        if (this == return_to_launch)
            return "failsafe return to launch"
        if (this == stop_offboard)
            return "auto stop offboard mode"
        if (this == start_manual)
            return "start manual mode"
        if (this == stop_manual)
            return "stop manual mode"
        return value
    }


    @JsonCreator
    static StateMsgEnum fromValue(String text) {
        for (StateMsgEnum b : values()) {
            if (String.valueOf(b.value) == text) {
                return b
            }
        }
        return unknown
    }
}