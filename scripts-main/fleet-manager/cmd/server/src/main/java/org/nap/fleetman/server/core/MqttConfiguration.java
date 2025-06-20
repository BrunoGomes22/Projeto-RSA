package org.nap.fleetman.server.core;

import org.eclipse.paho.client.mqttv3.IMqttClient;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class MqttConfiguration {
	private static final Logger log = LoggerFactory.getLogger(MqttConfiguration.class);

	@Bean
	@ConfigurationProperties(prefix = "mqtt")
	public MqttConnectOptions mqttConnectOptions() {
		MqttConnectOptions options = new MqttConnectOptions();
		options.setAutomaticReconnect(true);
		options.setCleanSession(true);
		options.setConnectionTimeout(10);
		return options;
	}

	@Bean
	public IMqttClient mqttClient(@Value("${mqtt.clientId:groundstation}") String clientId,
								  @Value("${mqtt.hostname:localhost}") String hostname, @Value("${mqtt.port:1883}") int port) {
		IMqttClient mqttClient = null;
		try {
			mqttClient = new MqttClient("tcp://" + hostname + ":" + port, clientId);
			mqttClient.connect(mqttConnectOptions());
		} catch (MqttException e) {
			log.error("Failed to connect to mqtt server on tcp://" + hostname + ":" + port + " - " + e.getMessage());
		}
		return mqttClient;
	}

}