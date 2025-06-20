package org.nap.fleetman.server.model.telemetry;

import com.fasterxml.jackson.annotation.JsonProperty;
import org.springframework.validation.annotation.Validated;

import javax.persistence.Embeddable;
import javax.validation.Valid;
import java.util.Objects;

/**
 * Position or velocity in north-east-down coordinates
 */
@Validated
@Embeddable
public class NedCoordinates {
	@JsonProperty("north")
	private Float north = null;

	@JsonProperty("east")
	private Float east = null;

	@JsonProperty("down")
	private Float down = null;

	public NedCoordinates() {
	}

	public NedCoordinates(float north, float east, float down) {
		this.north = north;
		this.east = east;
		this.down = down;
	}

	@Valid
	public Float getNorth() {
		return north;
	}

	public void setNorth(Float north) {
		this.north = north;
	}

	@Valid
	public Float getEast() {
		return east;
	}

	public void setEast(Float east) {
		this.east = east;
	}

	@Valid
	public Float getDown() {
		return down;
	}

	public void setDown(Float down) {
		this.down = down;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) {
			return true;
		}
		if (o == null || getClass() != o.getClass()) {
			return false;
		}
		NedCoordinates position = (NedCoordinates) o;
		return Objects.equals(this.north, position.north) &&
				Objects.equals(this.east, position.east) &&
				Objects.equals(this.down, position.down);
	}

	@Override
	public int hashCode() {
		return Objects.hash(north, east, down);
	}

	@Override
	public String toString() {
		return "north: " + north + ", east: " + east + ", down: " + down;
	}
}
