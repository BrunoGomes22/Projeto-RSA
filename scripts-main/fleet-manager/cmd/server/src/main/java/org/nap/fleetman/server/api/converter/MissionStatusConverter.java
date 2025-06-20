package org.nap.fleetman.server.api.converter;

import org.nap.fleetman.server.model.mission.MissionStatus;
import org.springframework.core.convert.converter.Converter;

public class MissionStatusConverter implements Converter<String, MissionStatus> {
	@Override
	public MissionStatus convert(String source) {
		return MissionStatus.valueOf(source.toUpperCase());
	}
}
