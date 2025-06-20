package org.nap.fleetman.server.model.telemetry;

import com.fasterxml.jackson.annotation.JsonProperty;
import org.springframework.validation.annotation.Validated;

import javax.persistence.Embeddable;
import javax.validation.Valid;
import java.util.Objects;

/**
 * Relative position the drone is moving to
 */
@Validated
@Embeddable
public class RelativePosition {
	@JsonProperty("forward")
	private Double forward = null;

	@JsonProperty("right")
	private Double right = null;

	@JsonProperty("up")
	private Double up = null;

	public RelativePosition() {
	}

	public RelativePosition(double forward, double right, double up) {
		this.forward = forward;
		this.right = right;
		this.up = up;
	}

	@Valid
	public Double getForward() {
		return forward;
	}

	public void setForward(Double forward) {
		this.forward = forward;
	}

	@Valid
	public Double getRight() {
		return right;
	}

	public void setRight(Double right) {
		this.right = right;
	}

	@Valid
	public Double getUp() {
		return up;
	}

	public void setUp(Double up) {
		this.up = up;
	}


	@Override
	public boolean equals(Object o) {
		if (this == o) {
			return true;
		}
		if (o == null || getClass() != o.getClass()) {
			return false;
		}
		RelativePosition position = (RelativePosition) o;
		return Objects.equals(this.forward, position.forward) &&
				Objects.equals(this.right, position.right) &&
				Objects.equals(this.up, position.up);
	}

	@Override
	public int hashCode() {
		return Objects.hash(forward, right, up);
	}

	@Override
	public String toString() {
		return "forward: " + forward + ", right: " + right + ", up: " + up;
	}
}
