package org.nap.fleetman.server.api;

import org.springframework.stereotype.Component;
import org.springframework.web.filter.AbstractRequestLoggingFilter;
import javax.servlet.http.HttpServletRequest;
import java.io.IOException;
import java.util.stream.Collectors;

@Component
public class RequestLoggingFilter extends AbstractRequestLoggingFilter {
	public RequestLoggingFilter() {
		this.setIncludeQueryString(true);
		this.setIncludePayload(true);
		this.setMaxPayloadLength(10000);
		this.setIncludeHeaders(false);
	}

	protected boolean shouldLog(HttpServletRequest request) {
		return !request.getMethod().equals("GET");
	}

	// TODO customize messages for all requests
	protected void beforeRequest(HttpServletRequest request, String message) {
		if (request.getMethod().equals("POST") && request.getRequestURI().equals("/mission"))
			this.logger.info("Received request [POST /mission]");
		else
			this.logger.info(message.replace("\n", "").replace("\t", "").replace("\r", "").replace("  ", "").replace("Before", "Received"));
	}

	protected void afterRequest(HttpServletRequest request, String message) {}
}
