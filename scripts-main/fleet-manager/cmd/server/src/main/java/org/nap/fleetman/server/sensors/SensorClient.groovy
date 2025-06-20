package org.nap.fleetman.server.sensors

import org.nap.fleetman.server.core.DroneManager
import org.nap.fleetman.server.exceptions.MissionException
import org.nap.fleetman.server.model.mission.MissionEndMsg
import org.springframework.stereotype.Component

@Component
class SensorClient {
    private SensorManager sensorMan
    private DroneManager droneMan

    SensorClient(SensorManager sensorMan, DroneManager droneMan) {
        this.sensorMan = sensorMan
        this.droneMan = droneMan
    }

    // The instance of this class can be invoked in a mission binding as "sensor".
    // Calling "sensor.{property}" in a mission script will call this method with {property} as an argument.
    def propertyMissing(String property) {
        // If the property is an existing drone ID, return a map with this drone's sensor values sorted by sensor type
        if (droneMan.droneExists(property))
            sensorMan.getDroneSensorsData(property)
        // If the property is a sensor type that requires a drone, return a map with sensors values sorted by drone ID
        else if (sensorMan.sensorTypeExistsAndRequiresDrone(property))
            sensorMan.getTypeSensorsDataByDrone(property)
        // If the property is another sensor type, return the corresponding sensor values for this type
        else if (sensorMan.sensorTypeExists(property))
            sensorMan.getTypeSensorsData(property)
        else
            throw new MissionException(MissionEndMsg.SENSOR_UNKNOWN, "Requested data from unknown sensor '$property'")
    }
}
