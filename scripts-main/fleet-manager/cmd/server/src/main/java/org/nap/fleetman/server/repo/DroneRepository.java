package org.nap.fleetman.server.repo;

import org.nap.fleetman.server.model.drone.Drone;
import org.nap.fleetman.server.model.drone.DroneState;
import org.springframework.data.jpa.repository.EntityGraph;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface DroneRepository extends JpaRepository<Drone, String> {
	@EntityGraph(value = "drone.graph", type = EntityGraph.EntityGraphType.LOAD)
	Drone findByDroneId(String droneId);

	@EntityGraph(value = "drone.graph", type = EntityGraph.EntityGraphType.LOAD)
	List<Drone> findByState(DroneState state);

	@EntityGraph(value = "drone.graph", type = EntityGraph.EntityGraphType.LOAD)
	List<Drone> findByStateAndMission(DroneState state, String mission);

	@EntityGraph(value = "drone.graph", type = EntityGraph.EntityGraphType.LOAD)
	List<Drone> findAll();
}