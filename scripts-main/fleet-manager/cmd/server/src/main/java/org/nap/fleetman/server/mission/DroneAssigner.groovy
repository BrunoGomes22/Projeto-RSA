package org.nap.fleetman.server.mission

import org.nap.fleetman.server.api.NotFoundException
import org.nap.fleetman.server.core.DroneManager
import org.nap.fleetman.server.exceptions.DroneAssignmentException
import org.nap.fleetman.server.exceptions.InstructionParsingException
import org.nap.fleetman.server.exceptions.MissionException
import org.nap.fleetman.server.mission.dsl.PositionCategory
import org.nap.fleetman.server.model.drone.Drone
import org.nap.fleetman.server.model.drone.DroneState
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.model.dsl.Requirement
import org.nap.fleetman.server.model.mission.MissionEndMsg
import org.nap.fleetman.server.model.telemetry.Position
import org.nap.fleetman.server.sensors.SensorManager
import org.springframework.stereotype.Service

@Service
class DroneAssigner {
    private MissionManager missionMan
    private DroneManager droneMan
    private WrapperManager wrapperMan
    private SensorManager sensorMan

    DroneAssigner(MissionManager missionMan, DroneManager droneMan, WrapperManager wrapperMan, SensorManager sensorMan) {
        this.missionMan = missionMan
        this.droneMan = droneMan
        this.wrapperMan = wrapperMan
        this.sensorMan = sensorMan
    }

    def revoke(DroneWrapper... drones) {
        if (drones == null)
            throw new MissionException(MissionEndMsg.REVOKED, "Request to revoke non-active drone reference")
        drones.each { drone -> missionMan.revokeDroneFromMission(drone.id) }
    }

    def replace(DroneWrapper drone) {
        if (!missionMan.canReplaceDrone(drone.id))
            throw new MissionException(MissionEndMsg.IRREPLACEABLE, "Cannot replace drone that was assigned as required for this mission")
        String droneId = assignReuse(drone.position)
        missionMan.replaceDroneAssignment(drone, droneId)
    }

    private assignReuse(Position pos) {
        def availableDrones = getAvailableDrones(Requirement.any)
        if (availableDrones.isEmpty())
            throw new DroneAssignmentException(DroneAssignmentException.State.REPLACE)
        String droneId = pickDrone(availableDrones, pos)
        return droneId
    }

    def assign(Object... requirements) {
        def wrappers = Arrays.asList(requirements)
        def sortedReqs = sortAssignmentPriority(requirements)

        sortedReqs.each { req ->
            if (req.class == String && !sensorMan.sensorTypeExistsAndRequiresDrone(req))
                wrappers.set(wrappers.indexOf(req), assignDroneToMission(req, req))
            else {
                List availableDrones = getAvailableDrones(req)
                if (availableDrones.isEmpty())
                    throw new DroneAssignmentException(DroneAssignmentException.State.REQUIREMENT, "$req")
                else
                    wrappers.set(wrappers.indexOf(req), assignDroneToMission(pickDrone(availableDrones, reqIsCoord(req) ? req : null), req))
            }
        }

        wrappers.size() > 1 ? wrappers : wrappers.get(0)
    }

    private assignDroneToMission(String droneId, req) {
        // FIXME refactor this method of requirement signaling
        if (req instanceof String) {
            if (sensorMan.sensorTypeExistsAndRequiresDrone(req))
                req = [req]
            else req = new LinkedList<>()
        }
        else
            req = null

        try {
            def dw = missionMan.addDroneToMission(droneId, req)
            if (dw == null)
                throw new DroneAssignmentException(DroneAssignmentException.State.UNAVAILABLE, droneId)
            else
                return dw
        } catch (NotFoundException e) {
            throw new DroneAssignmentException(DroneAssignmentException.State.NOT_REGISTERED, droneId)
        }
    }

    // FIXME requirements should be defined more clearly
    private sortAssignmentPriority(reqs) {
        reqs.sort { x, y ->
            if (x.class == y.class) {
                if (x instanceof List)
                    return y.size() - x.size()
                return 0
            }
            if (x.class == String && !sensorMan.sensorTypeExistsAndRequiresDrone(x))
                return -1
            if (y.class == String && !sensorMan.sensorTypeExistsAndRequiresDrone(y))
                return 1
            if (x instanceof List)
                return -1
            if (y instanceof List)
                return 1
            if (x.class == String) // Sensor
                return -1
            if (y.class == String) // Sensor
                return 1
            if (reqIsCoord(x))
                return -1
            if (reqIsCoord(y))
                return 1
            if (x == Requirement.any)
                return -1
            if (y == Requirement.any)
                return 1
        }
    }

    private static boolean reqIsCoord(Object o) {
        o instanceof Map || o instanceof Position
    }

    private getAvailableDrones(Object req) {
        List<Drone> availableDrones

        if (reqIsCoord(req)) {
            if (req instanceof Map && !(req.containsKey("lat") && req.containsKey("lon")))
                throw new InstructionParsingException(InstructionParsingException.State.NOT_COORDINATES)
            req = Requirement.any
        }

        switch (req.class) {
            case Requirement:
                availableDrones = droneMan.getAvailableDrones(null)
                break
            case String:
                availableDrones = droneMan.getAvailableDrones([req])
                break
            case List:
                Set<String> sensors = new HashSet<String>(req)
                if (sensors.size() == 0)
                    throw new InstructionParsingException(InstructionParsingException.State.EMPTY_SENSORS)
                sensors.each {sensor ->
                    if (!sensorMan.sensorTypeExistsAndRequiresDrone(sensor))
                        throw new InstructionParsingException(InstructionParsingException.State.UNRECOGNIZED_SENSORS, "$sensor")
                }
                availableDrones = droneMan.getAvailableDrones(sensors.asList())
                break
            default:
                throw new InstructionParsingException(InstructionParsingException.State.UNRECOGNIZED_REQ, "$req")
                return
        }
        availableDrones
    }

    private String pickDrone(List<Drone> availableDrones, coord = null) {
        def minSensorNumber = availableDrones.min { it.sensors.size() }.sensors.size()
        def leastSensorsDrones = availableDrones.findAll { it.sensors.size() == minSensorNumber }

        if (coord != null)
            return availableDrones.min { drone -> use(PositionCategory) { droneMan.getTelemetry(drone.droneId).position.distance(lat: coord.lat, lon: coord.lon) } }.droneId
        else if (leastSensorsDrones.size() == 1)
            return leastSensorsDrones.get(0).droneId
        else {
            return leastSensorsDrones.max { droneMan.getTelemetry(it.droneId).battery.percentage }.droneId
        }
    }
}
