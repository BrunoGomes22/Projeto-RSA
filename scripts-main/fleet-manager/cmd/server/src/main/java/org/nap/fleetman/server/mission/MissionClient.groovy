package org.nap.fleetman.server.mission

import groovyx.gpars.GParsPool
import org.nap.fleetman.server.concurrent.SyncChannel
import org.nap.fleetman.server.core.DroneManager
import org.nap.fleetman.server.exceptions.MissionException
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.model.dsl.Time
import org.nap.fleetman.server.model.dsl.TimeUnit
import org.nap.fleetman.server.model.mission.MissionEndMsg
import org.nap.fleetman.server.sensors.SensorManager
import org.springframework.stereotype.Service

// Stop mission
// Get all drones from this mission
// Assign/replace?
// Set as required drone
// log mission
// Move droneManager actions to here
@Service
class MissionClient {
    private SyncChannel syncChannel
    private TaskLauncher taskLauncher
    private SensorManager sensorMan
    private DroneManager droneMan

    MissionClient(SyncChannel syncChannel, TaskLauncher taskLauncher, SensorManager sensorMan, DroneManager droneMan) {
        this.syncChannel = syncChannel
        this.taskLauncher = taskLauncher
        this.sensorMan = sensorMan
        this.droneMan = droneMan
    }

    // Methods to wait for a fixed amount of time
    def wait(Time time) {
        Thread.sleep(time.convertTo(TimeUnit.millisecond).longValue())
    }

    // Assumes time is provided in milliseconds
    def wait(Long time) {
        Thread.sleep(time)
    }

    // Assumes time is provided in milliseconds
    def wait(Integer time) {
        Thread.sleep(time)
    }

    // TODO most mission exceptions are thrown as caused by Script error; should more appropriate end messages be created?
    // Wait for events: telemetry update, sensor update, task conclusion
    def wait(Map args) {
        // Only one event type per call
        if (args.keySet().size() != 1)
            throw new MissionException(MissionEndMsg.SCRIPT, "Waiting must be done on a single event type, found: ${args.keySet()}")
        String type = args.keySet()[0]
        def subject = args.values()[0]
        switch (type) {
            // Wait for telemetry message
            case 'telem':
                if (subject instanceof List)
                    GParsPool.withPool { subject.eachParallel { drone -> waitForTelem(drone) } }
                else
                    waitForTelem(subject)
                break
            // Wait for task conclusion
            case 'task':
                def missionId = MissionManager.thisMissionId
                taskLauncher.wait(subject, missionId)
                break
            // Wait for the conclusion of multiple tasks (parallel waiting is accomplished by taskLauncher)
            case 'tasks':
                def missionId = MissionManager.thisMissionId
                GParsPool.withPool { subject.eachParallel { task -> taskLauncher.wait(task, missionId) } }
                break
            // Wait for sensor message
            case 'sensor':
                waitForSensor(subject)
                break
            // Wait for message of multiple sensors
            case 'sensors':
                GParsPool.withPool { subject.eachParallel { sensor -> waitForSensor(sensor) } }
                break
            default:
                throw new MissionException(MissionEndMsg.SCRIPT, "Waiting event types can be 'drone(s)', 'task(s)', and 'sensor(s)', found '$type'")
        }
    }

    private waitForTelem(drone) {
        if (drone instanceof DroneWrapper) {
            syncChannel.waitTelem(drone)
        } else {
            throw new MissionException(MissionEndMsg.SCRIPT, "Provided unknown object as drone")
        }
    }

    private waitForSensor(sensor) {
        if (!(sensor instanceof Map))
            throw new MissionException(MissionEndMsg.SCRIPT, "Sensor must be described as a tuple")
        if (!sensor.containsKey("type"))
            throw new MissionException(MissionEndMsg.SCRIPT, "Sensor description must contain 'type'")
        if (!sensorMan.sensorTypeExists(sensor.type))
            throw new MissionException(MissionEndMsg.SENSOR_UNKNOWN, "Requested to wait for unknown sensor type '${sensor.type}'")
        def unknownKeys = sensor.keySet().findAll { key -> key != 'type' && key != 'id' && key != 'drone'}
        if (!unknownKeys.isEmpty())
            throw new MissionException(MissionEndMsg.SCRIPT, "Sensor description can only contain 'type', 'id', and/or 'drone', found $unknownKeys")
        if (sensor.containsKey('drone') && sensor.drone instanceof DroneWrapper)
            sensor.drone = sensor.drone.id
        syncChannel.waitSensor(sensor.type, sensor.id, sensor.drone)
    }
}
