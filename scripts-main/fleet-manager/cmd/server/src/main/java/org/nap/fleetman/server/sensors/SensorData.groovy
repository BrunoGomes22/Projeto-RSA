package org.nap.fleetman.server.sensors

import com.fasterxml.jackson.annotation.JsonIgnore
import com.fasterxml.jackson.databind.annotation.JsonSerialize
import groovy.transform.ToString
import org.nap.fleetman.server.api.TimestampSerializer
import org.springframework.data.annotation.Id
import org.springframework.data.mongodb.core.index.CompoundIndex
import org.springframework.data.mongodb.core.mapping.Document

@Document
@ToString
@CompoundIndex(def = "{'type':1, 'sensorId':1, 'droneId':1}", name = "sensor_index")
class SensorData {
    @Id
    private String id
    String type
    String sensorId
    String droneId
    def value
    @JsonSerialize(using = TimestampSerializer.class)
    Long timestamp

    SensorData() {}

    SensorData(String type, String sensorId, String droneId, value, Long timestamp, boolean persistAllData=false) {
        this.type = type.toLowerCase()
        this.sensorId = sensorId
        this.droneId = droneId
        this.value = value
        this.timestamp = timestamp
        if (!persistAllData)
            id = type+sensorId+droneId
    }

    @JsonIgnore
    String getId() {
        return id
    }
}
