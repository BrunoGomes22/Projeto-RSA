package org.nap.fleetman.server.api;

import org.springframework.boot.actuate.endpoint.annotation.ReadOperation;
import org.springframework.boot.actuate.endpoint.web.annotation.EndpointWebExtension;
import org.springframework.boot.actuate.logging.LogFileWebEndpoint;
import org.springframework.core.io.Resource;
import org.springframework.lang.Nullable;
import org.springframework.stereotype.Component;
import org.springframework.web.servlet.resource.TransformedResource;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.List;

@Component
@EndpointWebExtension(endpoint = LogFileWebEndpoint.class)
public class LogEndpointExtension {
	private final LogFileWebEndpoint delegate;

	public LogEndpointExtension(LogFileWebEndpoint delegate) {
		this.delegate = delegate;
	}

	@ReadOperation(produces = {"text/plain; charset=UTF-8"})
	public Resource logFile(@Nullable List<String> droneId, @Nullable List<String> logLevel) throws IOException {
		Resource resource = delegate.logFile();
		BufferedReader br = new BufferedReader(new InputStreamReader(resource.getInputStream()));
		StringBuilder output = new StringBuilder();
		while (br.ready()) {
			String log = br.readLine();
			String[] logSplit = log.split(" ");
			if ((droneId == null || droneId.contains(logSplit[3])) && (logLevel == null || logLevel.stream().anyMatch(logSplit[2]::equalsIgnoreCase)))
				output.append(log).append("\n");
		}
		return new TransformedResource(resource, output.toString().getBytes());
	}
}
