package org.nap.fleetman.server.model.telemetry;

import com.fasterxml.jackson.annotation.JsonProperty;
import org.springframework.validation.annotation.Validated;

import javax.persistence.Embeddable;
import javax.validation.Valid;
import java.util.Objects;

/**
 * Drone's latest known coordinates
 */
@Validated
@Embeddable
public class Position {
	@JsonProperty("lat")
	private Double lat = null;

	@JsonProperty("lon")
	private Double lon = null;

	@JsonProperty("alt")
	private Float alt = null;

	public Position() {
	}

	public Position(double lat, double lon, float alt) {
		this.lat = lat;
		this.lon = lon;
		this.alt = alt;
	}

	@Valid
	public Double getLat() {
		return lat;
	}

	public void setLat(Double lat) {
		this.lat = lat;
	}

	@Valid
	public Double getLon() {
		return lon;
	}

	public void setLon(Double lon) {
		this.lon = lon;
	}

	@Valid
	public Float getAlt() {
		return alt;
	}

	public void setAlt(Float alt) {
		this.alt = alt;
	}


	@Override
	public boolean equals(java.lang.Object o) {
		if (this == o) {
			return true;
		}
		if (o == null || getClass() != o.getClass()) {
			return false;
		}
		Position gpSCoordinates = (Position) o;
		return Objects.equals(this.lat, gpSCoordinates.lat) &&
				Objects.equals(this.lon, gpSCoordinates.lon) &&
				Objects.equals(this.alt, gpSCoordinates.alt);
	}

	@Override
	public int hashCode() {
		return Objects.hash(lat, lon, alt);
	}

	@Override
	public String toString() {
		return "lat: " + lat + ", lon: " + lon + ", alt: " + alt;
	}
}
