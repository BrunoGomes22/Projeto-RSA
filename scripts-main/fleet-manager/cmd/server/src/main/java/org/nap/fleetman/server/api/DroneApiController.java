package org.nap.fleetman.server.api;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.nap.fleetman.server.core.DroneManager;
import org.nap.fleetman.server.core.MessagePublisher;
import org.nap.fleetman.server.exceptions.ParameterException;
import org.nap.fleetman.server.mission.MissionManager;
import org.nap.fleetman.server.model.drone.Drone;
import org.nap.fleetman.server.model.drone.DroneState;
import org.nap.fleetman.server.model.mission.MissionEndMsg;
import org.nap.fleetman.server.model.telemetry.Telemetry;
import org.nap.fleetman.server.parameters.ParameterTracker;
import org.nap.fleetman.server.utils.CommandValidator;
import org.nap.fleetman.server.utils.DroneLog;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;
import rcl_interfaces.msg.SetParametersResult;

import javax.servlet.http.HttpServletRequest;
import javax.validation.Valid;
import java.time.Instant;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

@Controller
@CrossOrigin(origins = "*")
public class DroneApiController {

	private final ObjectMapper mapper;
	private final HttpServletRequest request;
	private final DroneManager droneManager;
	private final MissionManager missionManager;
	private final MessagePublisher messagePublisher;
	private final ParameterTracker paramTracker;

	public DroneApiController(ObjectMapper mapper, HttpServletRequest request, DroneManager droneManager,
							  MissionManager missionManager, MessagePublisher messagePublisher, ParameterTracker parameterTracker) {
		this.mapper = mapper;
		this.request = request;
		this.droneManager = droneManager;
		this.missionManager = missionManager;
		this.messagePublisher = messagePublisher;
		this.paramTracker = parameterTracker;
	}

	@RequestMapping(value = "/drone/{droneId}/cmd", consumes = {"application/json"}, method = RequestMethod.POST)
	public ResponseEntity<Void> droneDroneIdCmdPost(@Valid @RequestBody String body, @PathVariable("droneId") String droneId) {
		// Check if json structure represents a well-formed command
		try {
			ObjectNode cmd = mapper.readValue(body, ObjectNode.class);
			if (!CommandValidator.validateJsonCommand(mapper, cmd))
				return new ResponseEntity<>(HttpStatus.BAD_REQUEST);

			// Publish command to a ROS2 topic
			boolean isReturnToLaunch = cmd.get("cmd").asText().equals("return");
			if (isReturnToLaunch) {
				Drone drone = droneManager.getDrone(droneId);
				if (drone.getMission() != null) {
					DroneLog.warn(droneId, "Received return to launch command while assigned to mission");
					missionManager.cancelMission(drone.getMission(), MissionEndMsg.USER);
				}
			} else if (!droneManager.canDroneExecuteCommand(droneId, null))
				return new ResponseEntity<>(HttpStatus.CONFLICT);
			
			// print the command to the console
			messagePublisher.publishCommand(mapper.writeValueAsString(cmd.put("droneId", droneId).put("timestamp", Instant.now().toEpochMilli())));
			return new ResponseEntity<>(HttpStatus.CREATED);
		} catch (NotFoundException e) {
			return new ResponseEntity<>(HttpStatus.NOT_FOUND);
		} catch (JsonProcessingException e) {
			return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
		}
	}

	@RequestMapping(value = "/drone/{droneId}", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<Object> droneDroneIdGet(@PathVariable("droneId") String droneId,
												  @Valid @RequestParam(value = "data", required = false) String data) {

		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json")) {
			try {
				Drone drone = droneManager.getDrone(droneId);
				if (data == null) {
					Telemetry telem = droneManager.getTelemetry(droneId);
					ObjectNode root = mapper.createObjectNode();
					root.set("info", mapper.valueToTree(drone));
					root.set("telem", mapper.valueToTree(telem));
					return new ResponseEntity<>(root, HttpStatus.OK);
				}
				if (data.equals("telem")) {
					return new ResponseEntity<>(droneManager.getTelemetry(droneId), HttpStatus.OK);
				}
				if (data.equals("info"))
					return new ResponseEntity<>(drone, HttpStatus.OK);
			} catch (NotFoundException e) {
				return new ResponseEntity<>(HttpStatus.NOT_FOUND);
			}
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/drone", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<Object> droneGet(
			@Valid @RequestParam(value = "state", required = false) DroneState state,
			@Valid @RequestParam(value = "sensorTypes", required = false) List<String> sensorTypes,
			@Valid @RequestParam(value = "data", required = false) String data) {

		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json")) {
			List<Drone> drones = droneManager.getDrones(state, sensorTypes);
			if (drones.size() == 0)
				return new ResponseEntity<>(new LinkedList<>(), HttpStatus.OK);
			try {
				if (data == null) {
					ArrayNode root = mapper.createArrayNode();
					for (Drone drone : drones) {
						ObjectNode node = mapper.createObjectNode();
						node.set("info", mapper.valueToTree(drone));
						try {
							node.set("telem", mapper.valueToTree(droneManager.getTelemetry(drone.getDroneId())));
						} catch (NotFoundException e) {
							node.set("telem", mapper.valueToTree(new Telemetry(drone.getDroneId())));
						}
						root.add(node);
					}
					return new ResponseEntity<>(root, HttpStatus.OK);
				} else if (data.equals("info")) {
					return new ResponseEntity<>(drones, HttpStatus.OK);
				} else if (data.equals("telem")) {
					List<Telemetry> telem = new LinkedList<>();
					for (Drone drone : drones) {
						telem.add(droneManager.getTelemetry(drone.getDroneId()));
					}
					return new ResponseEntity<>(telem, HttpStatus.OK);
				}
			} catch (NotFoundException e) {
				e.printStackTrace();
				return new ResponseEntity<>(HttpStatus.INTERNAL_SERVER_ERROR);
			}
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	// Update drone parameters
	@RequestMapping(value = "/drone/{droneId}/params", consumes = {"application/json"}, method = RequestMethod.PATCH)
	public ResponseEntity<Object> droneDroneIdParamsPatch(@RequestBody String body, @PathVariable("droneId") String droneId) {
		if (!droneManager.droneExists(droneId))
			return new ResponseEntity<>(new LinkedList<>(), HttpStatus.NOT_FOUND);
		Map<String, Object> params;
		try {
			params = mapper.readValue(body, HashMap.class);
		} catch (JsonProcessingException e) {
			Map<String, String> response = new HashMap<>();
			response.put("parameter", null);
			response.put("cause", "Failed to parse JSON body");
			return new ResponseEntity<>(response, HttpStatus.BAD_REQUEST);
		}
		SetParametersResult result = null;
		try {
			result = paramTracker.setParametersAndGetResult(droneId, params);
		} catch (ParameterException e) {
			Map<String, String> response = new HashMap<>();
			response.put("parameter", e.getParam());
			response.put("cause", e.getMessage());
			return new ResponseEntity<>(response, HttpStatus.BAD_REQUEST);
		}
		if (!result.getSuccessful()) {
			Map<String, String> response = new HashMap<>();
			response.put("parameter", null);
			response.put("cause", result.getReason());
			return new ResponseEntity<>(response, HttpStatus.BAD_REQUEST);
		}
		return new ResponseEntity<>(HttpStatus.NO_CONTENT);
	}

	// Retrieve drone parameters
	@RequestMapping(value = "/drone/{droneId}/params", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<Object> droneDroneIdParamsGet(@PathVariable("droneId") String droneId) {
		if (!droneManager.droneExists(droneId))
			return new ResponseEntity<>(HttpStatus.NOT_FOUND);
		return new ResponseEntity<>(paramTracker.getParameters(droneId), HttpStatus.OK);
	}

	// Retrieve drone parameter
	@RequestMapping(value = "/drone/{droneId}/params/{param}", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<Object> droneDroneIdParamsParamGet(@PathVariable("droneId") String droneId,
															 @PathVariable("param") String param) {
		if (!droneManager.droneExists(droneId))
			return new ResponseEntity<>(droneId, HttpStatus.NOT_FOUND);
		if (!paramTracker.parameterExists(droneId, param))
			return new ResponseEntity<>(param, HttpStatus.NOT_FOUND);
		return new ResponseEntity<>(paramTracker.getParameter(droneId, param), HttpStatus.OK);
	}
}
