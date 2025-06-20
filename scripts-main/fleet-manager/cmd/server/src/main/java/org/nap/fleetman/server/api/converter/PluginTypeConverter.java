package org.nap.fleetman.server.api.converter;

import org.nap.fleetman.server.plugins.PluginType;
import org.springframework.core.convert.converter.Converter;

public class PluginTypeConverter implements Converter<String, PluginType> {
	@Override
	public PluginType convert(String source) {
		return PluginType.valueOf(source.toUpperCase());
	}
}
