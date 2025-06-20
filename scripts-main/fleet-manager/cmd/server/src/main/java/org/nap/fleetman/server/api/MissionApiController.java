package org.nap.fleetman.server.api;

import io.micrometer.core.instrument.util.IOUtils;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.core.type.TypeReference;
import org.nap.fleetman.server.core.DroneManager;
import org.nap.fleetman.server.mission.MissionManager;
import org.nap.fleetman.server.mission.ScriptLoader;
import org.nap.fleetman.server.model.mission.MissionResumeAction;
import org.nap.fleetman.server.model.mission.Mission;
import org.nap.fleetman.server.model.mission.MissionEndMsg;
import org.nap.fleetman.server.model.mission.MissionStatus;
import org.nap.fleetman.server.model.drone.Drone;
import org.nap.fleetman.server.model.telemetry.Telemetry;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;

import javax.servlet.http.HttpServletRequest;
import javax.validation.Valid;
import java.io.InputStream;
import java.util.List;
import java.util.Map;

@Controller
@CrossOrigin(origins = "*")
public class MissionApiController {

	private static final Logger log = LoggerFactory.getLogger(MissionApiController.class);
	private final ObjectMapper mapper;
	private final HttpServletRequest request;
	private final ScriptLoader scriptLoader;
	private final DroneManager droneManager;
	private final MissionManager missionManager;

	public MissionApiController(ObjectMapper mapper, HttpServletRequest request, ScriptLoader scriptLoader, DroneManager droneManager, MissionManager missionManager) {
		this.mapper = mapper;
		this.request = request;
		this.scriptLoader = scriptLoader;
		this.droneManager = droneManager;
		this.missionManager = missionManager;
	}

	@RequestMapping(value = "/mission", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<List<Mission>> missionGet(@Valid @RequestParam(value = "status", required = false) MissionStatus status) {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json")) {
			List<Mission> missions;
			if (status == null)
				missions = missionManager.getMissions();
			else
				missions = missionManager.getMissions(status);
			return new ResponseEntity<>(missions, HttpStatus.OK);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/mission/{missionId}", method = RequestMethod.DELETE)
	public ResponseEntity<Void> missionMissionIdDelete(@PathVariable("missionId") String missionId) {
		try {
			if (missionManager.cancelMission(missionId, MissionEndMsg.USER))
				return new ResponseEntity<>(HttpStatus.OK);
		} catch (NotFoundException e) {
			return new ResponseEntity<>(HttpStatus.NOT_FOUND);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/mission/{missionId}", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<Mission> missionMissionIdGet(@PathVariable("missionId") String missionId) {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json")) {
			Mission mission = missionManager.getMission(missionId);
			if (mission == null)
				return new ResponseEntity<>(HttpStatus.NOT_FOUND);
			return new ResponseEntity<>(mission, HttpStatus.OK);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/mission", produces = {"application/json"}, method = RequestMethod.POST)
	public ResponseEntity<String> missionPost(InputStream dataStream) {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json")) {
			String script = IOUtils.toString(dataStream);
			if (script.isEmpty() || script.replaceAll("\\s", "").isEmpty()) {
				log.warn("Rejected mission with empty script");
				return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
			}
			String missionId = scriptLoader.loadScript(script);
			return new ResponseEntity<>("{\"missionId\": \"" + missionId + "\"}", HttpStatus.CREATED);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/mission/{missionId}/pause", method = RequestMethod.POST)
	public ResponseEntity<Object> missionMissionIdPausePost(@PathVariable("missionId") String missionId) {
		try {
			List<Drone> pausedDrones = missionManager.pauseMission(missionId);
			if (pausedDrones != null) {
				ArrayNode root = mapper.createArrayNode();
				for (Drone drone : pausedDrones) {
					ObjectNode node = mapper.createObjectNode();
					node.set("info", mapper.valueToTree(drone));
					node.set("telem", mapper.valueToTree(droneManager.getTelemetry(drone.getDroneId())));
					root.add(node);
				}
				return new ResponseEntity<>(root, HttpStatus.OK);
			}
		} catch (NotFoundException e) {
			return new ResponseEntity<>(HttpStatus.NOT_FOUND);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/mission/{missionId}/resume", consumes = {"application/json"}, method = RequestMethod.POST)
	public ResponseEntity<Object> missionMissionIdResumePost(@RequestBody(required = false) String body, @PathVariable("missionId") String missionId) {
		try {
			MissionResumeAction resumeAction;
			if (body == null || body.isEmpty())
				resumeAction = MissionResumeAction.getDefault();
			else {
				Map<String, String> options = mapper.readValue(body, new TypeReference<Map<String, String>>() {});
				resumeAction = MissionResumeAction.fromValue(options.get("action"));	
			
				if (resumeAction == null) {
					Map<String, String> response = Map.of("error", "Invalid action");
					return new ResponseEntity<>(response, HttpStatus.BAD_REQUEST);
				}
			}	
				
			if (missionManager.resumeMission(missionId, resumeAction))
				return new ResponseEntity<>(HttpStatus.OK);
		} catch (NotFoundException e) {
			return new ResponseEntity<>(HttpStatus.NOT_FOUND);
		} catch (JsonProcessingException e) {
			Map<String, String> response = Map.of("error", "Invalid JSON");
			return new ResponseEntity<>(response, HttpStatus.BAD_REQUEST);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	// replace drone
	@RequestMapping(value = "/mission/{missionId}/replace/{droneId}/{newDroneId}", method = RequestMethod.POST)
	public ResponseEntity<Void> missionMissionIdReplacePost(@PathVariable("missionId") String missionId, @PathVariable("droneId") String droneId, @PathVariable("newDroneId") String newDroneId) {
		if (missionManager.canReplaceDrone(droneId)) {
			missionManager.replaceDrone(missionId, droneId, newDroneId);
			return new ResponseEntity<>(HttpStatus.OK);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}
}
