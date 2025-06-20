package org.nap.fleetman.server.model.telemetry;

import com.fasterxml.jackson.annotation.JsonAlias;
import com.fasterxml.jackson.annotation.JsonProperty;
import org.springframework.validation.annotation.Validated;

import javax.persistence.Embeddable;
import java.util.Objects;

/**
 * Battery status
 */
@Validated
@Embeddable
public class Battery {
	@JsonProperty("voltage")
	private Float voltage = null;

	@JsonProperty("percentage")
	@JsonAlias("remaining_percent")
	private Float percentage = null;

	public Battery() {
	}

	public Float getVoltage() {
		return voltage;
	}

	public void setVoltage(Float voltage) {
		this.voltage = voltage;
	}

	public Float getPercentage() {
		return percentage;
	}

	public void setPercentage(Float percentage) {
		this.percentage = percentage;
	}


	@Override
	public boolean equals(java.lang.Object o) {
		if (this == o) {
			return true;
		}
		if (o == null || getClass() != o.getClass()) {
			return false;
		}
		Battery droneBattery = (Battery) o;
		return Objects.equals(this.voltage, droneBattery.voltage) &&
				Objects.equals(this.percentage, droneBattery.percentage);
	}

	@Override
	public int hashCode() {
		return Objects.hash(voltage, percentage);
	}
}
