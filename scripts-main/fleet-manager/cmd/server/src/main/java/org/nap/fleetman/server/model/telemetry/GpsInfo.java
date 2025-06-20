package org.nap.fleetman.server.model.telemetry;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonValue;
import org.springframework.validation.annotation.Validated;

import javax.persistence.Embeddable;
import javax.validation.Valid;

/**
 * Gps information
 */
@Validated
@Embeddable
public class GpsInfo {
	@JsonProperty("satellites")
	private Integer satellites = null;

	@JsonProperty("fixType")
	private FixTypeEnum fixType = null;

	public enum FixTypeEnum {
		NO_GPS("no_gps"),
		NO_FIX("no_fix"),
		FIX_2D("fix_2d"),
		FIX_3D("fix_3d"),
		FIX_DGPS("fix_dgps"),
		RTK_FLOAT("rtk_float"),
		RTK_FIXED("rtk_fixed");

		private String value;

		FixTypeEnum(String value) {
			this.value = value;
		}

		@Override
		@JsonValue
		public String toString() {
			return String.valueOf(value);
		}

		@JsonCreator
		public static FixTypeEnum fromValue(String text) {
			for (FixTypeEnum b : FixTypeEnum.values()) {
				if (String.valueOf(b.value).equals(text)) {
					return b;
				}
			}
			return null;
		}
	}

	public GpsInfo() {
	}

	@Valid
	public Integer getSatellites() {
		return satellites;
	}

	public void setSatellites(Integer satellites) {
		this.satellites = satellites;
	}

	public FixTypeEnum getFixType() {
		return fixType;
	}

	public void setFixType(FixTypeEnum fixType) {
		this.fixType = fixType;
	}
}
