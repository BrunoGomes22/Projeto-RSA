package org.nap.fleetman.server.remote

import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.databind.annotation.JsonSerialize
import org.nap.fleetman.server.api.TimestampSerializer

import java.time.Instant

// Data Transfer Object for carrying remote task status
class RemoteTaskDTO {
    @JsonProperty("taskType")
    RemoteTaskType taskType
    @JsonProperty("droneId")
    String droneId
    @JsonProperty("state")
    RemoteTaskState state
    @JsonProperty("progress")
    Float progress
    @JsonProperty("start")
    @JsonSerialize(using = TimestampSerializer.class)
    Long start = null
    @JsonProperty("end")
    @JsonSerialize(using = TimestampSerializer.class)
    Long end = null

    RemoteTaskDTO(RemoteTaskType taskType, String droneId) {
        this.taskType = taskType
        this.droneId = droneId
        this.state = RemoteTaskState.running
        this.progress = 0
        this.start = Instant.now().toEpochMilli()
    }
}
