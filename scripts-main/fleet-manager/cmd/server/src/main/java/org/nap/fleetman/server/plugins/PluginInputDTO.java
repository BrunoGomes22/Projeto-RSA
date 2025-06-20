package org.nap.fleetman.server.plugins;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import org.springframework.validation.annotation.Validated;

/**
 * PluginDTO input variable
 */
@Validated
public class PluginInputDTO {
	@JsonProperty("property")
	private String property = null;

	@JsonProperty("type")
	private String type = null;

	@JsonInclude(JsonInclude.Include.NON_NULL)
	@JsonProperty("default")
	private String _default = null;

	public PluginInputDTO property(String property) {
		this.property = property;
		return this;
	}

	public String getProperty() {
		return property;
	}

	public void setProperty(String property) {
		this.property = property;
	}

	public PluginInputDTO type(String type) {
		this.type = type;
		return this;
	}

	public String getType() {
		return type;
	}

	public void setType(String type) {
		this.type = type;
	}

	public PluginInputDTO _default(String _default) {
		this._default = _default;
		return this;
	}

	public String getDefault() {
		return _default;
	}

	public void setDefault(String _default) {
		this._default = _default;
	}
}
