package org.nap.fleetman.server.plugins;

import com.fasterxml.jackson.annotation.JsonProperty;
import org.nap.fleetman.server.plugins.PluginType;
import org.springframework.validation.annotation.Validated;

import javax.validation.Valid;
import java.util.ArrayList;
import java.util.List;

/**
 * PluginDTO as retrieved from the system
 */
@Validated
public class PluginDTO {
	@JsonProperty("pluginId")
	private String pluginId = null;

	@JsonProperty("type")
	private PluginType type = null;

	@JsonProperty("input")
	@Valid
	private List<PluginInputDTO> input = null;

	@JsonProperty("missions")
	@Valid
	private List<String> missions = null;

	public PluginDTO pluginId(String pluginId) {
		this.pluginId = pluginId;
		return this;
	}

	public String getPluginId() {
		return pluginId;
	}

	public void setPluginId(String pluginId) {
		this.pluginId = pluginId;
	}

	public PluginDTO type(PluginType type) {
		this.type = type;
		return this;
	}

	@Valid
	public PluginType getType() {
		return type;
	}

	public void setType(PluginType type) {
		this.type = type;
	}

	public PluginDTO input(List<PluginInputDTO> input) {
		this.input = input;
		return this;
	}

	public PluginDTO addInputItem(PluginInputDTO inputItem) {
		if (this.input == null) {
			this.input = new ArrayList<>();
		}
		this.input.add(inputItem);
		return this;
	}

	@Valid
	public List<PluginInputDTO> getInput() {
		return input;
	}

	public void setInput(List<PluginInputDTO> input) {
		this.input = input;
	}

	public PluginDTO missions(List<String> missions) {
		this.missions = missions;
		return this;
	}

	public PluginDTO addMissionsItem(String missionsItem) {
		if (this.missions == null) {
			this.missions = new ArrayList<String>();
		}
		this.missions.add(missionsItem);
		return this;
	}

	public List<String> getMissions() {
		return missions;
	}

	public void setMissions(List<String> missions) {
		this.missions = missions;
	}
}
