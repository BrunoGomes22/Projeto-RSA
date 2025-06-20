package org.nap.fleetman.server.sensors

import groovy.io.FileType
import groovy.yaml.YamlSlurper
import org.nap.fleetman.server.concurrent.SyncChannel
import org.nap.fleetman.server.core.DroneManager
import org.nap.fleetman.server.mission.WrapperManager
import org.nap.fleetman.server.repo.SensorRepository
import org.nap.fleetman.server.utils.DroneLog
import org.nap.fleetman.server.utils.MissionUtils
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.beans.factory.annotation.Value
import org.springframework.stereotype.Service

import javax.annotation.PostConstruct
/**
 * Service for sensor management
 */
@Service
class SensorManager {
    private static final Logger log = LoggerFactory.getLogger(SensorManager.class)
    private DroneManager droneMan
    private WrapperManager wrapperMan
    private SyncChannel syncChannel
    @Value('${fleetman.sensors.dir:sensors}')
    private File configDirectory
    private final SensorRepository sensorRep
    private Map<String, SensorConfig> sensorConfiguration
    private Set<String> detectedSensors

    SensorManager(DroneManager droneMan, WrapperManager wrapperMan, SyncChannel syncChannel, SensorRepository sensorRep) {
        this.droneMan = droneMan
        this.wrapperMan = wrapperMan
        this.syncChannel = syncChannel
        this.sensorRep = sensorRep
        sensorConfiguration = new HashMap<>()
        detectedSensors = new HashSet<>()
    }

    // Load sensor configurations that are stored in the provided sensor configuration directory on application startup
    @PostConstruct
    private loadSensorConfigs() {
        if (configDirectory.exists()) {
            configDirectory.eachFileRecurse(FileType.FILES) { file ->
                file.withReader { reader ->
                    SensorConfig cfg = parseSensorConfig(reader, file.name)
                    if (cfg != null) {
                        cfg.file = file
                        sensorConfiguration.put(cfg.type, cfg)
                        log.info("Loaded ${cfg.type} sensor configuration from file '$file.name'")
                    }
                }
            }
        } else {
            log.warn("Provided sensor configuration directory does not exist.")
        }
    }

    // Adds new sensor configuration
    Map<String, Object> addSensorConfig(String config) {
        SensorConfig cfg = parseSensorConfig(config)
        // If the parsed configuration is null, the configuration is invalid
        if (cfg == null)
            return [type: null, success: false]
        // There can only be on sensor configuration per type
        if (sensorConfiguration.containsKey(cfg.type)) {
            log.error("Sensor type '$cfg.type' already exists")
            return [type: cfg.type, success: false]
        }
        // Save this sensor configuration to a file
        cfg.file = MissionUtils.writeConfigToFile(config, configDirectory, cfg.type, 'yml')
        sensorConfiguration.put(cfg.type, cfg)
        log.info("Loaded ${cfg.type} sensor configuration")
        return [type: cfg.type, success: true]
    }

    // Adds a new sensor configuration or updates already existent configuration
    // Returns integer result code: 0 - updated, 1 - created, 2 - parsing errors, 3 - provided ID and plugin ID mismatch
    int addOrUpdateSensorConfig(String type, String config) {
        SensorConfig cfg = parseSensorConfig(config)
        if (cfg == null)
            return 2
        if (type != cfg.type) {
            log.error("Provided sensor type '$type' does not match the sensor configuration's type '${cfg.type}'")
            return 3
        }
        // If this sensor type was already configured, replace it
        if (sensorConfiguration.containsKey(type)) {
            SensorConfig prevCfg = sensorConfiguration[type]
            cfg.file = MissionUtils.writeConfigToFile(config, configDirectory, type, 'yml', prevCfg.file)
            sensorConfiguration.put(type, cfg)
            log.info("Reloaded $type sensor configuration")
            return 0
        }
        // Otherwise, create a new sensor configuration
        cfg.file = MissionUtils.writeConfigToFile(config, configDirectory, type, 'yml')
        sensorConfiguration.put(type, cfg)
        log.info("Loaded $type sensor configuration")
        return 1
    }

    // Parse YAML file containing sensor configuration
    private static SensorConfig parseSensorConfig(config, String fileName=null) {
        Map yamlCfg
        try {
            if (fileName == null)
                yamlCfg = new YamlSlurper().parseText(config)
            else
                yamlCfg = new YamlSlurper().parse(config)
        } catch (Exception ignored) {
            log.error("Failed to load sensor configuration${fileName == null ? "" : " from file '$fileName'"}: provided invalid configuration")
            return null
        }
        // Validate sensor configuration
        try {
            return SensorValidator.validateSensorConfig(yamlCfg)
        } catch (IllegalArgumentException e) {
            log.error("Failed to load ${yamlCfg.type == null ? "" : "$yamlCfg.type "}sensor configuration: $e.message")
            return null
        }
    }

    // Return all sensor configurations
    List<SensorConfig> getSensorConfigs() {
        return sensorConfiguration.values() as List
    }

    // Return a specific sensor type's configuration
    SensorConfig getSensorConfig(String type) {
        def cfg = sensorConfiguration.find { _, cfg -> cfg.type == type.toLowerCase() }
        return cfg ? cfg.value : null
    }

    // Retrieve the most recent data for the detected sensors
    List<SensorData> getSensorData(List<String> type, List<String> sensorId, List<String> droneId) {
        // Sensor type must be lower case
        type = type.collect { t -> t.toLowerCase() }
        // If no filter is provided, return all sensor data
        if (type.isEmpty() && sensorId.isEmpty() && droneId.isEmpty())
            return sensorRep.findAll()
        // If all filters are provided, return the requested data
        if (!type.isEmpty() && !sensorId.isEmpty() && !droneId.isEmpty())
            return sensorRep.findByTypeInAndSensorIdInAndDroneIdIn(type, sensorId, droneId)
        // Retrieve sensor data according to the provided filters
        // Having a JPA query method for each combination is more efficient when retrieving from the database
        if (type.isEmpty()) {
            if (sensorId.isEmpty())
                return sensorRep.findByDroneIdIn(droneId)
            if (droneId.isEmpty())
                return sensorRep.findBySensorIdIn(sensorId)
            return sensorRep.findBySensorIdInAndDroneIdIn(sensorId, droneId);
        }
        if (droneId.isEmpty()) {
            if (sensorId.isEmpty())
                return sensorRep.findByTypeIn(type)
            return sensorRep.findByTypeInAndSensorIdIn(type, sensorId)
        }
        return sensorRep.findByTypeInAndDroneIdIn(type, droneId)
    }

    // Return sensors that have exceeded their timeout limit
    List<SensorData> getTimedOutSensors() {
        List<SensorData> data = []
        // Find all sensor configurations which have a defined timeout
        sensorConfiguration.findAll { _, cfg -> cfg.timeout != null && cfg.timeout > 0 }.each { _, cfg ->
            // Retrieve sensor data in which the latest entry has happened before the time limit
            if (cfg.requiresDrone)
                data += sensorRep.findByDroneIdIsNotNullAndTypeAndTimestampLessThan(cfg.type, System.currentTimeMillis() - cfg.timeout)
            else
                data += sensorRep.findByDroneIdIsNullAndTypeAndTimestampLessThan(cfg.type, System.currentTimeMillis() - cfg.timeout)
        }
        return data
    }

    // Get all sensor data for a specific drone sorted by sensor type
    Map getDroneSensorsData(String droneId) {
        Map sensorsData = [:]
        droneMan.getDrone(droneId).sensors.each { type ->
            // Get all sensor data for this drone
            def currentSensorType = getSensorsByDroneAndType(droneId, type)
            // Format the data according to this sensor type's retrieval strategy
            sensorsData[type] = retrieveSensorDataValue(sensorConfiguration[type].retrieval, currentSensorType)
        }
        return sensorsData
    }

    // Get all sensor data for this sensor type
    def getTypeSensorsData(String type) {
        return retrieveSensorDataValue(sensorConfiguration[type].retrieval, getSensorsByType(type))
    }

    // Get all sensor data for a sensor type that requires drone, sorted by drone ID
    Map getTypeSensorsDataByDrone(String type) {
        // Get all sensor data for this type
        List<SensorData> sensorsData = getSensorsByType(type)
        Map sensorsDataByDrone = [:]
        // Collect all drone IDs that have a sensor of this type
        sensorsData.collect { sensor -> sensor.droneId }.each { droneId ->
            def sensors = sensorsData.findAll { sensor -> sensor.droneId == droneId }
            // Format the data according to the retrieval strategy by drone ID
            sensorsDataByDrone[droneId] = retrieveSensorDataValue(sensorConfiguration[type].retrieval, sensors)
        }
        return sensorsDataByDrone
    }

    // Return sensor value according to the sensor type's retrieval strategy
    private static retrieveSensorDataValue(SensorConfig.RetrievalStrategy retrievalStrategy, List<SensorData> sensors) {
        switch (retrievalStrategy) {
            case SensorConfig.RetrievalStrategy.LATEST:
                SensorData latest = sensors[0]
                return latest.value
            case SensorConfig.RetrievalStrategy.LIST:
                return sensors.value
            case SensorConfig.RetrievalStrategy.MAP:
                return convertSensorDataListToValueMap(sensors)
        }
    }

    private List<SensorData> getSensorsByDroneAndType(String droneId, String type) {
        return sensorRep.findByDroneIdAndTypeOrderByTimestampDesc(droneId, type)
    }

    private List<SensorData> getSensorsByType(String type) {
        return sensorRep.findByTypeOrderByTimestampDesc(type)
    }

    // Convert sensor data list to a map with sensor ID as key
    private static Map convertSensorDataListToValueMap(List<SensorData> dataList) {
        dataList.collectEntries { sensor -> [(sensor.sensorId): sensor.value] }
    }

    // Update latest sensor data
    void updateSensor(Map data) {
        if (data.type == null) {
            log.error("Received sensor message without type")
            return
        }
        // Retrieve configuration for this sensor
        SensorConfig cfg = sensorConfiguration[data.type]
        if (cfg == null) {
            log.error("Received sensor message with unknown type '$data.type'")
            return
        }
        // Validate sensor data
        SensorData sensor
        try {
            sensor = SensorValidator.validateSensorData(data, cfg)
        } catch (IllegalArgumentException e) {
            log.error("Received invalid $data.type sensor message: $e.message")
            return
        }
        // Save sensor data to database
        sensorRep.save(sensor)
        // Notify waiting threads that new sensor data is available
        syncChannel.unlockSensor(sensor.type, sensor.sensorId, sensor.droneId, false)
        // Log if this sensor was detected for the first time
        if (!detectedSensors.contains(sensor.id)) {
            detectedSensors.add(sensor.id)
            String logString = "Detected $sensor.type sensor${cfg.requiresSensorId ? " with id '$sensor.sensorId'" : ""}"
            // If this sensor is attached to a drone, add sensor type to the drone's list of available sensors
            if (cfg.requiresDrone) {
                droneMan.addSensorToDrone(sensor.droneId, sensor.type)
                DroneLog.info(sensor.droneId, logString)
            } else log.info(logString)
        }
    }

    // Check if lowercase type is registered
    boolean sensorTypeExists(String type) {
        sensorConfiguration[type.toLowerCase()] != null
    }

    // Check if lowercase type is registered and if this sensor type requires a drone
    boolean sensorTypeExistsAndRequiresDrone(String type) {
        SensorConfig cfg = sensorConfiguration[type.toLowerCase()]
        return cfg != null && cfg.requiresDrone
    }

    // Remove sensor from detected sensors if it had timed out
    boolean removeSensor(SensorData sensor) {
        if (detectedSensors.contains(sensor.id)) {
            // Remove database entry
            sensorRep.delete(sensor)
            // Notify waiting threads that this sensor is disconnected
            syncChannel.unlockSensor(sensor.type, sensor.sensorId, sensor.droneId, true)
            // Remove from detected sensors list
            detectedSensors.remove(sensor.id)
            String logString = "${sensor.type.capitalize()} sensor ${sensorConfiguration[sensor.type].requiresSensorId ? "with id '$sensor.sensorId' " : ""}timed out"
            // If sensor is attached to drone, remove it from the drone's available sensor list
            if (sensor.droneId != null && (sensor.sensorId == null || getSensorsByDroneAndType(sensor.droneId, sensor.type).isEmpty())) {
                droneMan.removeSensorFromDrone(sensor.droneId, sensor.type)
                DroneLog.warn(sensor.droneId, logString)
            } else log.warn(logString)
            return true
        }
        return false
    }

    // Delete sensor type configuration
    boolean removeSensorConfig(String type) {
        type = type.toLowerCase()
        SensorConfig cfg = sensorConfiguration.remove(type)
        if (cfg == null)
            return false
        cfg.file.delete()
        log.info("Removed $type sensor configuration")
        return true
    }
}
