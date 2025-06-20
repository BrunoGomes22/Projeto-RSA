package org.nap.fleetman.server.api;

import com.fasterxml.jackson.databind.ObjectMapper;
import io.micrometer.core.instrument.util.IOUtils;
import org.nap.fleetman.server.core.DroneManager;
import org.nap.fleetman.server.sensors.SensorConfig;
import org.nap.fleetman.server.sensors.SensorManager;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.RequestParam;

import javax.servlet.http.HttpServletRequest;
import java.io.InputStream;
import java.util.List;
import java.util.Map;

@Controller
public class SensorApiController {
	private static final Logger log = LoggerFactory.getLogger(SensorApiController.class);
	private final ObjectMapper objectMapper;
	private final HttpServletRequest request;
	private final SensorManager sensorManager;
	private final DroneManager droneManager;

	public SensorApiController(ObjectMapper objectMapper, HttpServletRequest request, SensorManager sensorManager, DroneManager droneManager) {
		this.objectMapper = objectMapper;
		this.request = request;
		this.sensorManager = sensorManager;
		this.droneManager = droneManager;
	}

	@RequestMapping(value = "/sensor/config", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<List<SensorConfig>> sensorConfigGet() {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json"))
			return new ResponseEntity<>(sensorManager.getSensorConfigs(), HttpStatus.OK);
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/sensor/config", produces = {"application/json"}, consumes = {"text/yaml"}, method = RequestMethod.POST)
	public ResponseEntity<String> sensorConfigPost(InputStream dataStream) {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json")) {
			String config = IOUtils.toString(dataStream);
			if (config.isEmpty() || config.replaceAll("\\s", "").isEmpty()) {
				log.warn("Rejected sensor configuration with empty script");
				return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
			}
			Map<String, Object> result = sensorManager.addSensorConfig(config);
			if (result.get("type") == null)
				return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
			if (result.get("success").equals(false))
				return new ResponseEntity<>(HttpStatus.CONFLICT);
			return new ResponseEntity<>("{\"sensorType\": \"" + result.get("type") + "\"}", HttpStatus.CREATED);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/sensor/config/{type}", method = RequestMethod.DELETE)
	public ResponseEntity<Void> sensorConfigTypeDelete(@PathVariable("type") String type) {
		if (sensorManager.removeSensorConfig(type))
			return new ResponseEntity<>(HttpStatus.OK);
		return new ResponseEntity<>(HttpStatus.NOT_FOUND);
	}

	@RequestMapping(value = "/sensor/config/{type}", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<SensorConfig> sensorConfigTypeGet(@PathVariable("type") String type) {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json")) {
			SensorConfig cfg = sensorManager.getSensorConfig(type);
			if (cfg == null)
				return new ResponseEntity<>(HttpStatus.NOT_FOUND);
			else
				return new ResponseEntity<>(cfg, HttpStatus.OK);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/sensor/config/{type}", consumes = {"text/yaml"}, method = RequestMethod.PUT)
	public ResponseEntity<Void> sensorConfigTypePut(InputStream dataStream, @PathVariable("type") String type) {
		String config = IOUtils.toString(dataStream);
		if (config.isEmpty() || config.replaceAll("\\s", "").isEmpty()) {
			log.warn("Rejected sensor configuration with empty script");
			return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
		}
		int result = sensorManager.addOrUpdateSensorConfig(type, config);
		switch (result) {
			case 1:
				return new ResponseEntity<>(HttpStatus.CREATED);
			case 2:
				return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
			case 3:
				return new ResponseEntity<>(HttpStatus.CONFLICT);
		}
		return new ResponseEntity<>(HttpStatus.OK);
	}

	@RequestMapping(value = "/sensor", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<Object> sensorGet(
			@RequestParam(value = "type", required = false, defaultValue = "") List<String> type,
			@RequestParam(value = "sensorId", required = false, defaultValue = "") List<String> sensorId,
			@RequestParam(value = "droneId", required = false, defaultValue = "") List<String> droneId) {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json")) {
			for (String t : type) {
				if (!sensorManager.sensorTypeExists(t))
					return new ResponseEntity<>("Sensor type '" + t + "' does not exist", HttpStatus.NOT_FOUND);
			}
			for (String d : droneId) {
				if (!droneManager.droneExists(d))
					return new ResponseEntity<>("Drone with ID '" + d + "' does not exist", HttpStatus.NOT_FOUND);
			}
			return new ResponseEntity<>(sensorManager.getSensorData(type, sensorId, droneId), HttpStatus.OK);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}
}
