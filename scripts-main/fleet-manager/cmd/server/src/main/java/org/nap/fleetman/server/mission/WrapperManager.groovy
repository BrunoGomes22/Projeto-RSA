package org.nap.fleetman.server.mission

import org.nap.fleetman.server.concurrent.SyncChannel
import org.nap.fleetman.server.mission.dsl.PositionCategory
import org.nap.fleetman.server.model.drone.CommandEnum
import org.nap.fleetman.server.model.drone.Drone
import org.nap.fleetman.server.model.drone.DroneState
import org.nap.fleetman.server.model.dsl.DistanceUnit
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.model.telemetry.Telemetry
import org.nap.fleetman.server.model.mission.MissionResumeAction
import org.nap.fleetman.server.parameters.ParameterTracker
import org.nap.fleetman.server.remote.RemoteTaskHandler
import org.nap.fleetman.server.utils.CommandHandler
import org.nap.fleetman.server.utils.DroneLog
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.context.annotation.Lazy
import org.springframework.stereotype.Service

import java.util.concurrent.ConcurrentHashMap

@Service
class WrapperManager {
    private SyncChannel syncChannel
    private CommandHandler cmdHandler
    private ParameterTracker paramTracker
    private RemoteTaskHandler remoteTaskHandler
    private static final Logger log = LoggerFactory.getLogger(WrapperManager.class)
    private Map<String, DroneWrapper> wrappers

    WrapperManager(SyncChannel syncChannel, CommandHandler cmdHandler, ParameterTracker paramTracker,
                   @Lazy RemoteTaskHandler remoteTaskHandler) {
        this.syncChannel = syncChannel
        this.cmdHandler = cmdHandler
        this.paramTracker = paramTracker
        this.remoteTaskHandler = remoteTaskHandler
        wrappers = new ConcurrentHashMap<>()
    }

    synchronized DroneWrapper getDroneWrapper(String droneId) {
        return wrappers.get(droneId)
    }

    DroneWrapper createDroneWrapper(Drone drone, Telemetry telem) {
        def wrapper = new DroneWrapper(drone.droneId)
        // Link this drone's parameter Map to this DroneWrapper instance (when the original map is updated with new values,
        // this reference will also be updated)
        wrapper.setParams(paramTracker.getParameters(drone.droneId))
        wrapper.task = remoteTaskHandler.createDroneRemoteTaskReference(drone.droneId)
        wrappers.put(drone.droneId, wrapper)
        updateWrapperTelem(drone, telem)
        wrapper
    }

    DroneWrapper updateWrapperTelem(Drone drone, Telemetry telem) {
        DroneWrapper dw = getDroneWrapper(drone.droneId)
        if (dw != null && !dw.isBeingReplaced) {
            def prev_timestamp = dw.getTimestamp()
            dw.updateTelem(drone, telem)
            if (prev_timestamp != dw.getTimestamp())
                syncChannel.unlockTelem(drone.droneId)
        }
        dw
    }

    List<DroneWrapper> getDroneWrappers(Set<String> droneIds) {
        def drones = []
        droneIds.each { drone -> drones.add(wrappers.get(drone)) }
        drones
    }

    void revokeDrone(String droneId) {
        log.info("Mission: revoking drone '$droneId'")
        def dw = getDroneWrapper(droneId)
        if (dw == null) {
            log.debug("Requested to revoke drone wrapper with id '$droneId' that does not exist")
            return
        }
        revokeDroneWrapper(dw)
        wrappers.remove(droneId)
    }

    private revokeDroneWrapper(DroneWrapper drone) {
        drone.revoke()
        remoteTaskHandler.stopAndRemoveRemoteDroneTask(drone.id)
        DroneLog.info(drone.id, "Revoked from mission")
    }

    void withdrawDrone(String droneId) {
        DroneLog.info(droneId, "Withdrawing from mission")
        remoteTaskHandler.stopAndRemoveRemoteDroneTask(droneId)
        wrappers.remove(droneId)
    }

    def reassignDrone(DroneWrapper wrapper, String droneId, String missionId, MissionResumeAction action = MissionResumeAction.LAST_POSITION) {
        wrapper.isBeingReplaced = true
        def droneData = fetchDroneData(wrapper)
        updateWrapperReferences(wrapper, droneId)
        resumeCommand(wrapper, droneData, missionId, true, action)
        wrapper.isBeingReplaced = false
        DroneLog.info(droneId, "Reassigned to mission")
    }

    def replaceDrone(DroneWrapper wrapper, String newDroneId, String missionId) {
        wrapper.isBeingReplaced = true
        def prevDroneData = retirePreviousDrone(wrapper)
        switchWrapperReferences(wrapper, newDroneId)
        resumeCommand(wrapper, prevDroneData, missionId)
        wrapper.isBeingReplaced = false
    }

    Map fetchDroneData(String droneId) {
        def drone = getDroneWrapper(droneId)
        if (drone == null)
            return null
        fetchDroneData(drone)
    }

    Map fetchDroneData(DroneWrapper drone) {
        def data = [position: drone.position, home: drone.home, state: drone.state, heading: drone.heading, id: drone.id]
        if (drone.cmd != null) {
            data['cmd'] = drone.cmd.type
            data['target'] = drone.cmd.target
        }
        data
    }

    private Map retirePreviousDrone(DroneWrapper drone) {
        def prevDroneData = fetchDroneData(drone)
        if (drone.cmd == null) {
            if (prevDroneData.state == DroneState.READY)
                relocateDrone(drone)
            else
                returnDroneHome(drone)
            return prevDroneData
        }

        switch (drone.cmd.type) {
            case CommandEnum.return_to_launch:
                break
            case CommandEnum.turn:
                prevDroneData['cmd'] = null
                syncChannel.waitCommand(drone.id, drone.cmd.type as String)
                returnDroneHome(drone)
                break
            case CommandEnum.takeoff:
                if (distanceToHome(drone) > 1)
                    returnDroneHome(drone)
                else {
                    cmdHandler.cancel(drone.id)
                    cmdHandler.land(drone.id)
                }
                break
            case CommandEnum.land:
                if (distanceToHome(drone) > 1)
                    returnDroneHome(drone)
                else {
                    cmdHandler.cancel(drone.id)
                    relocateDrone(drone)
                }
                break
            default:
                cmdHandler.cancel(drone.id)
                returnDroneHome(drone)
                break
        }
        return prevDroneData
    }

    private updateWrapperReferences(DroneWrapper drone, String droneId) {
        DroneLog.info(drone.id, "Updating drone wrapper references")
        //wrappers.remove(drone.id)
        drone.setId(droneId)
        drone.setParams(paramTracker.getParameters(droneId))
        drone.task = remoteTaskHandler.createDroneRemoteTaskReference(droneId)
        drone.revoked = false
        wrappers.put(droneId, drone)
    }

    private switchWrapperReferences(DroneWrapper drone, String droneId) {
        revokeDroneWrapper(drone)
        wrappers.remove(drone.id)
        drone.setId(droneId)
        drone.setParams(paramTracker.getParameters(droneId))
        drone.task = remoteTaskHandler.createDroneRemoteTaskReference(droneId)
        drone.revoked = false
        wrappers.put(droneId, drone)
    }

    private resumeCommand(DroneWrapper drone, Map prevDroneData, String missionId, boolean inAir = false, MissionResumeAction action = MissionResumeAction.LAST_POSITION) {
        log.info("Mission \"$missionId\": resuming command for drone '$drone.id'")
        if (!inAir) {
            cmdHandler.arm(drone.id, true)
            cmdHandler.takeoff(drone.id, null, true)
        }
        Number takeoffAlt = null
        if (prevDroneData.state != DroneState.READY)
            takeoffAlt = prevDroneData.position.alt

        if (action == MissionResumeAction.LAST_POSITION) {
            cmdHandler.goTo(drone.id, prevDroneData.position.lat, prevDroneData.position.lon, takeoffAlt, null, prevDroneData.heading, true)
            log.info("Mission \"$missionId\": finish replacement of drone '$prevDroneData.id' with drone '$drone.id'")
        }

        if (action == MissionResumeAction.LAST_POSITION || action == MissionResumeAction.COMPLETE_CURRENT) {
            switch (prevDroneData.cmd as CommandEnum) {
                case CommandEnum.takeoff:
                    Number alt_diff = prevDroneData.home.alt + prevDroneData.target - drone.position.alt
                    if (alt_diff > 0.5)
                        cmdHandler.move(drone.id, 0, 0, alt_diff, null, true)
                    break
                case CommandEnum.land:
                    cmdHandler.land(drone.id, true)
                    break
                case CommandEnum.go_to:
                case CommandEnum.move:
                    cmdHandler.goTo(drone.id, prevDroneData.target.lat, prevDroneData.target.lon, prevDroneData.target.alt, null, prevDroneData.heading, true)
                    break
                default:
                    if (prevDroneData.state == DroneState.READY)
                        cmdHandler.land(drone.id, true)
                    break
            }
        }
    }

    private returnDroneHome(DroneWrapper drone) {
        Number alt_diff = drone.position.alt - drone.home.alt
        Number default_return_alt = drone.params.returnAltitude as Number
        cmdHandler.returnToLaunch(drone.id, alt_diff < default_return_alt ? default_return_alt : alt_diff, null)
    }

    private relocateDrone(DroneWrapper drone) {
        if (!drone.armed)
            cmdHandler.arm(drone.id, true)
        cmdHandler.takeoff(drone.id, null, true)
        cmdHandler.move(drone.id, 0, 3, 0, null, true)
        cmdHandler.land(drone.id)
    }

    private static Number distanceToHome(DroneWrapper drone) {
        return PositionCategory.distance([lat: drone.position.lat, lon: drone.position.lon],
                                         [lat: drone.home.lat, lon: drone.home.lon]).convertTo(DistanceUnit.meter)
    }
}
