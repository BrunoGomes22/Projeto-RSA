package org.nap.fleetman.server.core;

import org.nap.fleetman.server.utils.NetUtils;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.parameters.ParameterVariant;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import javax.annotation.PostConstruct;
import java.util.ArrayList;
import java.util.List;

/**
 * ROS2 node wrapper that can be managed by Spring boot
 */
@Component
public class GroundNode extends BaseComposableNode {
	private static final Logger log = LoggerFactory.getLogger(GroundNode.class);
	private final String name;
	@Value("${fleetman.subnetAddress:10.1.1.0/24}")
	private String netAddress;

	public GroundNode(@Value("${fleetman.ros2.node:ground}") String name) {
		super(name);
		this.name = name;
	}

	// Set ipAddress and macAddress ground station node parameters
	@PostConstruct
	private void initParameters() {
		List<String> addresses = NetUtils.getAddresses(netAddress);
		if (addresses.isEmpty()) {
			log.warn("Could not detect network interface within the '" + netAddress + "' subnet");
		} else {
			List<ParameterVariant> parameters = new ArrayList<>();
			parameters.add(new ParameterVariant("ipAddress", addresses.get(0)));
			parameters.add(new ParameterVariant("macAddress", addresses.get(1)));
			for (ParameterVariant parameter : parameters)
				node.declareParameter(parameter);
			node.setParameters(parameters);
		}
	}

	public String getName() {
		return name;
	}
}
