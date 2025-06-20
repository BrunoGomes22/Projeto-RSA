package org.nap.fleetman.server.model.telemetry

import com.fasterxml.jackson.annotation.JsonAlias
import com.fasterxml.jackson.annotation.JsonInclude
import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.databind.annotation.JsonSerialize
import org.nap.fleetman.server.api.TimestampSerializer
import org.springframework.validation.annotation.Validated

import javax.persistence.*
import javax.validation.Valid

import static javax.persistence.FetchType.LAZY

/**
 * Telemetry data
 */
@Validated
@Entity
@NamedEntityGraph(name = 'telem.graph',
        attributeNodes = @NamedAttributeNode('healthFailures'))
class Telemetry {
    @Id
    @JsonProperty("droneId")
    String droneId = null

    @JsonProperty("armed")
    Boolean armed = null

    @JsonProperty("position")
    @AttributeOverride(name = 'alt', column = @Column(name = 'position_alt'))
    @AttributeOverride(name = 'lat', column = @Column(name = 'position_lat'))
    @AttributeOverride(name = 'lon', column = @Column(name = 'position_lon'))
    Position position = null

    @JsonProperty("positionNED")
    @JsonAlias("position_ned")
    @AttributeOverride(name = 'north', column = @Column(name = 'position_north'))
    @AttributeOverride(name = 'east', column = @Column(name = 'position_east'))
    @AttributeOverride(name = 'down', column = @Column(name = 'position_down'))
    NedCoordinates positionNed = null

    @JsonProperty("velocityNED")
    @JsonAlias("velocity_ned")
    @AttributeOverride(name = 'north', column = @Column(name = 'velocity_north'))
    @AttributeOverride(name = 'east', column = @Column(name = 'velocity_east'))
    @AttributeOverride(name = 'down', column = @Column(name = 'velocity_down'))
    NedCoordinates velocityNed = null

    @JsonProperty("heading")
    Double heading = null

    @JsonProperty("speed")
    Float speed = null

    @JsonInclude(JsonInclude.Include.NON_NULL)
    @JsonProperty("height")
    Float height = null
    
    @JsonProperty("home")
    @AttributeOverride(name = "alt", column = @Column(name = "home_alt"))
    @AttributeOverride(name = "lat", column = @Column(name = "home_lat"))
    @AttributeOverride(name = "lon", column = @Column(name = "home_lon"))
    Position home = null

    @JsonProperty("flightMode")
    @JsonAlias("flight_mode")
    FlightMode flightMode = null

    @JsonProperty("landState")
    @JsonAlias("landed_state")
    LandState landState = null

    @JsonProperty("battery")
    Battery battery = null

    @JsonProperty("gpsInfo")
    GpsInfo gpsInfo = null

    @JsonProperty("healthFailures")
    @JsonAlias("healthFail")
    @Valid
    @ElementCollection(targetClass = Health.class, fetch = LAZY)
    List<Health> healthFailures = null

    @JsonProperty("timestamp")
    @JsonSerialize(using = TimestampSerializer.class)
    Long timestamp = null

    Telemetry() {
    }

    Telemetry(String droneId) {
        this.droneId = droneId
        position = new Position()
        positionNed = new NedCoordinates()
        velocityNed = new NedCoordinates()
        home = new Position()
        flightMode = FlightMode.UNKNOWN
        landState = LandState.UNKNOWN
        battery = new Battery()
        gpsInfo = new GpsInfo()
        healthFailures = new ArrayList<>()
    }

    @Override
    boolean equals(Object o) {
        if (this == o) {
            return true
        }
        if (o == null || getClass() != o.getClass()) {
            return false
        }
        Telemetry telemetry = (Telemetry) o
        return Objects.equals(this.droneId, telemetry.droneId)
    }

    @Override
    int hashCode() {
        return Objects.hash(droneId)
    }
}
