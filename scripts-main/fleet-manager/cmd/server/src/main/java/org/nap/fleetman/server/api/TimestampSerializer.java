package org.nap.fleetman.server.api;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;

import java.io.IOException;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

public class TimestampSerializer extends StdSerializer<Long> {

	public TimestampSerializer() {
		this(null);
	}

	public TimestampSerializer(Class<Long> t) {
		super(t);
	}

	@Override
	public void serialize(Long aLong, JsonGenerator jsonGenerator, SerializerProvider serializerProvider) throws IOException {
		LocalDateTime timestamp = Instant.ofEpochMilli(aLong).atZone(ZoneId.of("Europe/Lisbon")).toLocalDateTime();
		jsonGenerator.writeString(timestamp.format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss,SSS")));
	}
}
