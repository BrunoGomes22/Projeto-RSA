package org.nap.fleetman.server.model.drone

import com.fasterxml.jackson.annotation.JsonIgnore
import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.databind.annotation.JsonSerialize
import org.nap.fleetman.server.api.TimestampSerializer
import org.nap.fleetman.server.model.telemetry.Position
import org.nap.fleetman.server.model.telemetry.RelativePosition
import org.springframework.validation.annotation.Validated

import javax.persistence.Embeddable
import javax.persistence.Transient

/**
 * Command
 */
@Validated
@Embeddable
class Command {
    @JsonIgnore
    CommandEnum type
    @JsonIgnore
    Position coords
    @JsonIgnore
    RelativePosition relativePos
    @JsonIgnore
    Float altitude
    @JsonIgnore
    Float degrees

    @JsonProperty("description")
    String description = null

    @JsonProperty("timestamp")
    @JsonSerialize(using = TimestampSerializer.class)
    Long timestamp = null

    Command(StatusMessage msg) {
        type = msg.command
        coords = msg.coords
        relativePos = msg.pos
        altitude = msg.altitude
        description = msg.commandToString().capitalize()
        timestamp = msg.timestamp
    }

    Command() {}

    @JsonIgnore
    def getTarget() {
        if (type == CommandEnum.go_to || type == CommandEnum.move)
            return coords
        if (type == CommandEnum.return_to_launch || type == CommandEnum.takeoff)
            return altitude
        if (type == CommandEnum.turn)
            return degrees
        return null
    }

    @Override
    boolean equals(Object o) {
        if (o == null)
            return false
        if (o.getClass() == CommandEnum.class)
            return Objects.equals(this.type, o)
        if (getClass() != o.getClass())
            return false
        Command command = (Command) o
        return Objects.equals(this.description, command.description)
    }
}
