package org.nap.fleetman.server;

import org.nap.fleetman.server.api.converter.DroneStateConverter;
import org.nap.fleetman.server.api.converter.MissionStatusConverter;
import org.nap.fleetman.server.api.converter.PluginTypeConverter;
import org.nap.fleetman.server.core.GroundNode;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executors.MultiThreadedExecutor;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.ConfigurableApplicationContext;
import org.springframework.format.FormatterRegistry;
import org.springframework.scheduling.annotation.EnableScheduling;

import javax.annotation.PreDestroy;

@SpringBootApplication
@EnableScheduling
public class FleetmanServerApplication {
	public static void main(String[] args) {
		// Initialize RCLJava
		RCLJava.rclJavaInit();
		ConfigurableApplicationContext ctx = SpringApplication.run(FleetmanServerApplication.class, args);
		// Register custom converters
		FormatterRegistry reg = ctx.getBean(FormatterRegistry.class);
		reg.addConverter(new DroneStateConverter());
		reg.addConverter(new MissionStatusConverter());
		reg.addConverter(new PluginTypeConverter());
		// Spin node
		GroundNode node = ctx.getBean(GroundNode.class);
		MultiThreadedExecutor exec = new MultiThreadedExecutor();
		exec.addNode(node);
		exec.spin();
	}

	@PreDestroy
	public void destroy() {
		RCLJava.shutdown();
	}
}
