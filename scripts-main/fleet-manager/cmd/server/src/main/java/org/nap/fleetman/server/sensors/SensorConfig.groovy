package org.nap.fleetman.server.sensors

import com.fasterxml.jackson.annotation.JsonIgnore
import groovy.transform.ToString
import groovy.transform.TupleConstructor

@ToString
@TupleConstructor
class SensorConfig {
    // LATEST - return most recent data of this sensor type
    // LIST - return a list with data from all sensors of this type
    // MAP - return a map with data from sensors of this type
    enum RetrievalStrategy { LATEST, LIST, MAP }
    // Allowed parameter types
    enum ParameterType {INT, LONG, DOUBLE, FLOAT, BOOLEAN, STRING, LIST}

    String type
    boolean requiresDrone
    boolean requiresSensorId
    RetrievalStrategy retrieval
    def parameters
    Long timeout
    @JsonIgnore
    File file
}
