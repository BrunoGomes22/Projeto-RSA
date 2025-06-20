package org.nap.fleetman.server.repo;

import org.nap.fleetman.server.model.telemetry.Telemetry;
import org.springframework.data.jpa.repository.EntityGraph;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface TelemRepository extends JpaRepository<Telemetry, String> {
	@EntityGraph(value = "telem.graph", type = EntityGraph.EntityGraphType.LOAD)
	Telemetry findByDroneId(String droneId);

	@EntityGraph(value = "telem.graph", type = EntityGraph.EntityGraphType.LOAD)
	List<Telemetry> findAllByOrderByTimestampAsc();

	@EntityGraph(value = "telem.graph", type = EntityGraph.EntityGraphType.LOAD)
	List<Telemetry> findAll();
}