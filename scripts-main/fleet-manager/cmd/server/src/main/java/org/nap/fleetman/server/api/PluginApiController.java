package org.nap.fleetman.server.api;

import io.micrometer.core.instrument.util.IOUtils;
import org.nap.fleetman.server.plugins.PluginDTO;
import org.nap.fleetman.server.plugins.PluginManager;
import org.nap.fleetman.server.plugins.PluginType;
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
public class PluginApiController {
	private static final Logger log = LoggerFactory.getLogger(PluginApiController.class);
	private final HttpServletRequest request;
	private final PluginManager pluginMan;

	public PluginApiController(HttpServletRequest request, PluginManager pluginMan) {
		this.request = request;
		this.pluginMan = pluginMan;
	}

	@RequestMapping(value = "/plugin", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<List<PluginDTO>> pluginGet(@Valid @RequestParam(value = "type", required = false) PluginType type) {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json"))
			return new ResponseEntity<>(pluginMan.getPluginDTOList(type), HttpStatus.OK);
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/plugin/{pluginId}", method = RequestMethod.DELETE)
	public ResponseEntity<Void> pluginPluginIdDelete(@PathVariable("pluginId") String pluginId) {
		if (pluginMan.removePlugin(pluginId))
			return new ResponseEntity<>(HttpStatus.OK);
		return new ResponseEntity<>(HttpStatus.NOT_FOUND);
	}

	@RequestMapping(value = "/plugin/{pluginId}", produces = {"application/json"}, method = RequestMethod.GET)
	public ResponseEntity<PluginDTO> pluginPluginIdGet(@PathVariable("pluginId") String pluginId) {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json"))
			return new ResponseEntity<>(pluginMan.getPluginDTO(pluginId), HttpStatus.OK);
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}

	@RequestMapping(value = "/plugin/{pluginId}", consumes = {"application/octet-stream"}, method = RequestMethod.PUT)
	public ResponseEntity<Void> pluginPluginIdPut(InputStream dataStream, @PathVariable("pluginId") String pluginId) {
		String script = IOUtils.toString(dataStream);
		if (script.isEmpty() || script.replaceAll("\\s", "").isEmpty()) {
			log.warn("Rejected plugin with empty script");
			return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
		}
		int result = pluginMan.createOrUpdatePlugin(pluginId, script);
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

	@RequestMapping(value = "/plugin", produces = {"application/json"}, consumes = {"application/octet-stream"}, method = RequestMethod.POST)
	public ResponseEntity<String> pluginPost(InputStream dataStream) {
		String accept = request.getHeader("Accept");
		if (accept != null && accept.contains("application/json")) {
			String script = IOUtils.toString(dataStream);
			if (script.isEmpty() || script.replaceAll("\\s", "").isEmpty()) {
				log.warn("Rejected plugin with empty script");
				return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
			}
			Map<String, Object> result = pluginMan.createPlugin(script);
			if (result.get("pluginId") == null)
				return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
			if (result.get("success").equals(false))
				return new ResponseEntity<>(HttpStatus.CONFLICT);
			return new ResponseEntity<>("{\"pluginId\": \"" + result.get("pluginId") + "\"}", HttpStatus.CREATED);
		}
		return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
	}
}
