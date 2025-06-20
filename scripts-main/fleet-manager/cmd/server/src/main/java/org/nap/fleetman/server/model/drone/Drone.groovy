package org.nap.fleetman.server.model.drone

import com.fasterxml.jackson.annotation.JsonInclude
import com.fasterxml.jackson.annotation.JsonProperty
import org.nap.fleetman.server.model.drone.Error
import org.nap.fleetman.server.model.drone.Warn
import org.nap.fleetman.server.remote.RemoteTaskType
import org.springframework.validation.annotation.Validated

import javax.persistence.*
// These imports are needed otherwise it will assume java.lang.Error at compile time

/**
 * Drone properties
 */
@Validated
@Entity
@NamedEntityGraph(name = 'drone.graph',
        attributeNodes = [@NamedAttributeNode('sensors'), @NamedAttributeNode('errors'), @NamedAttributeNode('warns')])
class Drone {
    @Id
    @JsonProperty("droneId")
    String droneId = null

    @JsonProperty("state")
    DroneState state = null

    @JsonProperty("currentCommand")
    Command currentCommand = null

    @JsonProperty("sensors")
    @ElementCollection(targetClass = String.class)
    Set<String> sensors = null

    @JsonProperty("mission")
    String mission = null

    @JsonProperty("errors")
    @ElementCollection(targetClass = Error.class)
    Set<Error> errors = null

    @JsonProperty("warns")
    @ElementCollection(targetClass = Warn.class)
    Set<Warn> warns = null

    @JsonProperty("remoteTask")
    @JsonInclude(JsonInclude.Include.NON_NULL)
    RemoteTaskType remoteTaskType = null

    Drone() {
        sensors = new LinkedHashSet<>()
        errors = new LinkedHashSet<>()
        warns = new LinkedHashSet<>()
    }

    Drone(String droneId) {
        this()
        this.droneId = droneId
        state = DroneState.UNKNOWN
    }

    void addSensor(String sensor) {
        sensors.add(sensor)
    }

    void removeSensor(String sensor) {
        sensors.remove(sensor)
    }

    void addError(Error error) {
        errors.add(error)
    }

    void removeError(Error error) {
        errors.remove(error)
    }

    void addWarn(Warn warn) {
        warns.add(warn)
    }

    void removeWarn(Warn warn) {
        warns.remove(warn)
    }

    @Override
    boolean equals(Object o) {
        if (this == o) {
            return true
        }
        if (o == null || getClass() != o.getClass()) {
            return false
        }
        Drone drone = (Drone) o
        return Objects.equals(this.droneId, drone.droneId)
    }
}
