package org.nap.fleetman.server.sensors

import org.codehaus.groovy.runtime.typehandling.GroovyCastException

class SensorValidator {

    static SensorConfig validateSensorConfig(Map cfg) {
        def missing = []
        if (cfg.type == null)
            missing += "type"
        if (cfg.retrieval == null)
            missing += "retrieval"
        if (cfg.parameters == null)
            missing += "parameters"

        if (!missing.isEmpty())
            throw new IllegalArgumentException("Missing value for $missing")

        return createSensorConfig(cfg)
    }

    private static SensorConfig createSensorConfig(Map cfg) {
        def type, requiresDrone, requiresSensorId, retrieval, parameters, timeout
        type = (cfg.type as String).toLowerCase()
        requiresDrone = cfg.requiresDrone ? cfg.requiresDrone as boolean : false
        requiresSensorId = cfg.requiresSensorId ? cfg.requiresSensorId as boolean : false
        try {
            retrieval = (cfg.retrieval as String).toUpperCase() as SensorConfig.RetrievalStrategy
        } catch (IllegalArgumentException ignored) {
            throw new IllegalArgumentException("Illegal value '$cfg.retrieval' for retrieval - existing strategies are latest, list and map")
        }
        if (retrieval == SensorConfig.RetrievalStrategy.MAP && !requiresSensorId)
            throw new IllegalArgumentException("Cannot use 'map' retrieval strategy without setting sensor ID as required")
        if (cfg.parameters instanceof Map) {
            parameters = cfg.parameters
            validateConfigParameter(parameters)
        } else {
            try {
                parameters = (cfg.parameters as String).toUpperCase() as SensorConfig.ParameterType
            } catch (IllegalArgumentException ignored) {
                throw new IllegalArgumentException("Illegal value '$cfg.parameters' as parameter - available types are int, long, double, float, boolean, string or list")
            }
        }
        try {
            timeout = cfg.timeout as Long
        } catch (IllegalArgumentException ignored) {
            throw new IllegalArgumentException("Illegal value '$cfg.timeout' for timeout")
        }
        return new SensorConfig(type, requiresDrone, requiresSensorId, retrieval, parameters, timeout)
    }

    private static void validateConfigParameter(cfg) {
        cfg.each {
            if (it.value instanceof Map) {
                validateConfigParameter(it.value)
            } else {
                try {
                    it.value = (it.value as String).toUpperCase() as SensorConfig.ParameterType
                } catch (IllegalArgumentException ignored) {
                    throw new IllegalArgumentException("Illegal value '$it.value' for parameter '$it.key' - available types are int, long, double, float, boolean, string or list")
                }
            }
        }
    }

    static SensorData validateSensorData(Map data, SensorConfig sensorConfig) {
        def missing = []
        if (data.type == null)
            missing += "type"
        if (data.value == null)
            missing += "value"
        if (data.timestamp == null)
            missing += "timestamp"
        if (sensorConfig.requiresDrone && data.droneId == null)
            missing += "droneId"
        if (sensorConfig.requiresSensorId && data.sensorId == null)
            missing += "sensorId"

        if (!missing.isEmpty())
            throw new IllegalArgumentException("Received sensor message ${data.type ? "from sensor type '$data.type' " : ""}with missing fields: $missing")

        return createSensorData(data, sensorConfig)
    }

    private static SensorData createSensorData(data, sensorConfig) {
        def type, sensorId, droneId, value, timestamp
        type = (data.type as String).toLowerCase()
        sensorId = data.sensorId ? (data.sensorId as String) : null
        droneId = data.droneId ? (data.droneId as String) : null
        if (data.value instanceof Map) {
            if (!(sensorConfig.parameters instanceof Map))
                throw new IllegalArgumentException("sensor value should be of type '${sensorConfig.parameters.toString().toLowerCase()}' but received parameters ${data.value.keySet()}")
            value = data.value
            def missing = []
            sensorConfig.parameters.keySet().each { param ->
                if (!(param in value.keySet()))
                    missing += param
            }
            if (!missing.isEmpty())
                throw new IllegalArgumentException("missing parameters: $missing")
            validateDataParameter(value, sensorConfig.parameters)
        } else {
            if (!(sensorConfig.parameters instanceof SensorConfig.ParameterType))
                throw new IllegalArgumentException("sensor value should contain '${sensorConfig.parameters.keySet()}' but received a single value")
            try {
                value = castDataParameterValue(data.value, sensorConfig.parameters)
            } catch (IllegalArgumentException | GroovyCastException ignored) {
                throw new IllegalArgumentException("value '$data.value' does not match configured type '${sensorConfig.parameters.toString().toLowerCase()}'")
            }
        }
        try {
            timestamp = data.timestamp as Long
        } catch (IllegalArgumentException ignored) {
            throw new IllegalArgumentException("Received invalid value '$data.timestamp' for timestamp")
        }
        return new SensorData(type, sensorId, droneId, value, timestamp)
    }

    private static void validateDataParameter(data, cfg) {
        data.each {
            if (it.value instanceof Map) {
                validateDataParameter(it.value, cfg[it.key])
            } else {
                try {
                    it.value = castDataParameterValue(it.value, cfg[it.key])
                } catch (IllegalArgumentException | GroovyCastException ignored) {
                    throw new IllegalArgumentException("value '$it.value' does not match configured type '${cfg[it.key].toString().toLowerCase()}' for parameter '$it.key'")
                }
            }
        }
    }

    private static castDataParameterValue(value, SensorConfig.ParameterType type) {
        switch (type) {
            case SensorConfig.ParameterType.INT:
                return value as Integer
            case SensorConfig.ParameterType.LONG:
                return value as Long
            case SensorConfig.ParameterType.DOUBLE:
                return value as Double
            case SensorConfig.ParameterType.FLOAT:
                return value as Float
            case SensorConfig.ParameterType.BOOLEAN:
                return value as Boolean
            case SensorConfig.ParameterType.STRING:
                return value as String
            case SensorConfig.ParameterType.LIST:
                return value as List
        }
    }
}
