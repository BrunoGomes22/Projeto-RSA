package org.nap.fleetman.server.repo;

import org.nap.fleetman.server.sensors.SensorData;
import org.springframework.data.mongodb.repository.MongoRepository;

import java.util.List;

public interface SensorRepository extends MongoRepository<SensorData, String> {
	List<SensorData> findAll();

	List<SensorData> findByTypeInAndSensorIdInAndDroneIdIn(List<String> type, List<String> sensorId, List<String> droneId);

	List<SensorData> findByTypeIn(List<String> type);

	List<SensorData> findBySensorIdIn(List<String> sensorId);

	List<SensorData> findByDroneIdIn(List<String> droneId);

	List<SensorData> findByTypeInAndSensorIdIn(List<String> type, List<String> sensorId);

	List<SensorData> findBySensorIdInAndDroneIdIn(List<String> sensorId, List<String> droneId);

	List<SensorData> findByTypeInAndDroneIdIn(List<String> type, List<String> droneId);


	List<SensorData> findByDroneIdAndTypeOrderByTimestampDesc(String droneId, String type);

	List<SensorData> findByTypeOrderByTimestampDesc(String type);

	List<SensorData> findByDroneIdIsNullAndTypeAndTimestampLessThan(String type, Long timestamp);

	List<SensorData> findByDroneIdIsNotNullAndTypeAndTimestampLessThan(String type, Long timestamp);

}