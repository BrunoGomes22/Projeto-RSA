package org.nap.fleetman.server.api.converter;

import org.nap.fleetman.server.model.drone.DroneState;
import org.springframework.core.convert.converter.Converter;

public class DroneStateConverter implements Converter<String, DroneState> {
	@Override
	public DroneState convert(String source) {
		return DroneState.valueOf(source.toUpperCase());
	}
}
