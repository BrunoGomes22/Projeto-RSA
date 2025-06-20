package org.nap.fleetman.server.mission

import org.nap.fleetman.server.api.NotFoundException
import org.nap.fleetman.server.concurrent.ThreadExecutor
import org.nap.fleetman.server.core.DroneManager
import org.nap.fleetman.server.model.drone.CommandEnum
import org.nap.fleetman.server.model.drone.Command
import org.nap.fleetman.server.model.drone.Drone
import org.nap.fleetman.server.model.drone.DroneState
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.remote.RemoteTaskAction
import org.nap.fleetman.server.remote.RemoteTaskDTO
import org.nap.fleetman.server.remote.RemoteTaskHandler
import org.nap.fleetman.server.remote.RemoteTaskState
import org.nap.fleetman.server.remote.RemoteTaskType
import org.nap.fleetman.server.model.mission.Mission
import org.nap.fleetman.server.model.mission.MissionEndMsg
import org.nap.fleetman.server.model.mission.MissionStatus
import org.nap.fleetman.server.model.mission.MissionResumeAction
import org.nap.fleetman.server.plugins.PluginManager
import org.nap.fleetman.server.utils.CommandHandler
import org.nap.fleetman.server.utils.MissionUtils
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Service

import java.time.Instant
import java.util.concurrent.ConcurrentHashMap

@Service
class MissionManager {
    private static final Logger log = LoggerFactory.getLogger(MissionManager.class)
    private DroneManager droneMan
    private WrapperManager wrapperMan
    private PluginManager pluginMan
    private CommandHandler cmdHandler
    private ThreadExecutor threadExecutor
    private Map<String, Mission> missions
    private Map<String, Thread> reassignJobs

    MissionManager(DroneManager droneMan, WrapperManager wrapperMan, PluginManager pluginMan, CommandHandler cmdHandler,
                   ThreadExecutor threadExecutor) {
        this.droneMan = droneMan
        this.wrapperMan = wrapperMan
        this.pluginMan = pluginMan
        this.cmdHandler = cmdHandler
        this.threadExecutor = threadExecutor
        missions = new ConcurrentHashMap<>()
        reassignJobs = new ConcurrentHashMap<>()
    }

    static String getThisMissionId() {
        Thread.currentThread().missionId
    }

    static String getThisThreadDroneId() {
        Thread.currentThread().droneId
    }

    static boolean isDroneThread() {
        Thread.currentThread().class == ThreadExecutor.MissionThread.class && Thread.currentThread().droneId != null
    }

    String initializeMission() {
        String missionId = MissionUtils.generateMissionId()
        log.info("Created mission with id \"" + missionId + "\"")
        Mission m = new Mission(missionId)
        m.start = System.currentTimeMillis()
        m.status = MissionStatus.PENDING
        missions.put(m.missionId, m)
        missionId
    }

    def startMission(String missionId) {
        Mission m = getMission(missionId)
        m.setStatus(MissionStatus.RUNNING)
        log.info("Mission \"$missionId\" started")
    }

    Mission getMission(String missionId) {
        missions.get(missionId)
    }

    List<Mission> getMissions(MissionStatus status) {
        List<Mission> matchingMissions = new LinkedList<>()
        missions.each {
            if (it.value.status == status)
                matchingMissions.add(it.value)
        }
        matchingMissions
    }

    List<Mission> getMissions() {
        missions.values().toList()
    }

    DroneWrapper addDroneToMission(String droneId, List<String> required = null) {
        addDroneToMission(droneId, thisMissionId, required)
    }

    DroneWrapper addDroneToMission(String droneId, String missionId, List<String> required = null) {
        // FIXME refactor this method of requirement signaling
        if (!droneMan.assignToMission(droneId, missionId))
            return null
        Mission m = missions.get(missionId)
        m.addActiveDrone(droneId)
        m.addUsedDrone(droneId)
        if (required != null)
            m.addRequiredDrone(droneId, required as Set)
        def drone = wrapperMan.createDroneWrapper(droneMan.getDrone(droneId), droneMan.getTelemetry(droneId))
        pluginMan.enablePluginForDrone(drone, missionId, m.plugins)
        drone
    }

    def reassignDroneToMission(String droneId, String missionId) {
        // if drone is from mission pausedDrones, reassign it to the mission
        Mission mission = getMission(missionId)
        if (isMissionPaused(missionId) && mission.pausedDrones.contains(droneId)) {
            if (!droneMan.assignToMission(droneId, missionId)) {
                log.error("Failed to reassign drone $droneId to mission $missionId")
                return null
            }

            mission.addActiveDrone(droneId)
            mission.removePausedDrone(droneId)

            log.info("Drone $droneId reassigned to mission $missionId")
        }
    }

    def addPluginToMission(String missionId, String pluginId) {
        Mission m = missions.get(missionId)
        m.addPlugin(pluginId)
        log.info("Mission \"$missionId\" enabled plugin \"$pluginId\"")
    }

    def removePluginFromMission(String missionId, String pluginId) {
        Mission m = missions.get(missionId)
        m.removePlugin(pluginId)
        log.info("Mission \"$missionId\" disabled plugin \"$pluginId\"")
    }

    def getMissionPlugins(String missionId=null) {
        if (missionId == null)
            missionId = thisMissionId
        missions.get(missionId).plugins
    }

    synchronized replaceDroneAssignment(DroneWrapper drone, String newDroneId) {
        String prevDroneId = drone.id
        Mission mission = getDroneMission(prevDroneId)
        if (!droneMan.assignToMission(newDroneId, mission.missionId))
            return null
        log.info("Mission \"$mission.missionId\": start replacement of drone '$prevDroneId' with drone '$newDroneId'")
        mission.addActiveDrone(newDroneId).addUsedDrone(newDroneId)
        wrapperMan.replaceDrone(drone, newDroneId, mission.missionId)
        mission.removeActiveDrone(prevDroneId)
        droneMan.clearDroneMission(prevDroneId)
    }

    synchronized concludeSuccessfulMission(String missionId) {
        concludeMission(missionId, MissionStatus.FINISHED, MissionEndMsg.SUCCESS)
        if (threadExecutor.missionHasOtherThreadsRunning(missionId))
            threadExecutor.stopMissionThreadsExceptThis(missionId)
    }

    synchronized boolean cancelMission(String missionId, MissionEndMsg msg) throws NotFoundException {
        if (concludeMission(missionId, MissionStatus.CANCELLED, msg)) {
            threadExecutor.stopMissionThreadsExceptThis(missionId)
            return true
        }
        return false
    }

    synchronized concludeFailedMission(MissionEndMsg msg, String logMsg, String pluginId=null) {
        // This is also checked here to avoid duplicate error log messages when concurrent errors are detected
        if (isMissionRunning(thisMissionId)) {
            if (pluginId != null) {
                log.error("Plugin \"${pluginId}\" runtime error: $msg - $logMsg")
                concludeFailedMission(MissionEndMsg.PLUGIN_ERROR)
            } else {
                log.error("Mission \"${thisMissionId}\": $logMsg")
                concludeFailedMission(msg)
            }
        }
    }

    synchronized concludeFailedMission(MissionEndMsg msg) {
        concludeMission(thisMissionId, MissionStatus.FAILED, msg)
        threadExecutor.stopMissionThreads(thisMissionId)
    }

    synchronized concludeFailedMission(String missionId, MissionEndMsg msg, String logMsg) throws NotFoundException {
        log.error("Mission \"${missionId}\": $logMsg")
        concludeFailedMission(missionId, msg)
    }

    synchronized concludeFailedMission(String missionId, MissionEndMsg msg) throws NotFoundException {
        concludeMission(missionId, MissionStatus.FAILED, msg)
        threadExecutor.stopMissionThreadsExceptThis(missionId)
    }

    synchronized private boolean concludeMission(String missionId, MissionStatus status, MissionEndMsg msg) {
        if (!isMissionRunning(missionId))
            return false
        Mission mission = missions.get(missionId)
        mission.status = status
        if (msg != MissionEndMsg.SUCCESS)
            mission.cause = msg
        mission.end = System.currentTimeMillis()
        mission.activeDrones.each { droneId ->
            droneMan.clearDroneMission(droneId)
            mission.addUsedDrone(droneId)
            Drone drone = droneMan.getDrone(droneId)
            if (drone.state == DroneState.ACTIVE && drone.currentCommand.type == CommandEnum.go_to)
                cmdHandler.cancel(droneId)
            wrapperMan.revokeDrone(droneId)
        }
        mission.clearActiveDrones()
        mission.plugins.each { pluginId -> pluginMan.disablePlugin(pluginId, mission.missionId) }
        if (status == MissionStatus.FAILED)
            log.warn("Mission \"${mission.missionId}\" concluded with status: $status - $msg")
        else
            log.info("Mission \"${mission.missionId}\" concluded with status: $status - $msg")
        return true
    }

    synchronized List<Drone> pauseMission(String missionId) throws NotFoundException {
        List<Drone> pausedDronesData = new LinkedList<>()
        Mission mission = getMission(missionId)
        if (isMissionRunning(missionId)) {
            mission.status = MissionStatus.PAUSED
            threadExecutor.pauseMissionThreads(missionId)
            
            // free all active drones
            mission.activeDrones.each { droneId ->
                // save drone state (with clone to preserve state)
                DroneWrapper drone = wrapperMan.getDroneWrapper(droneId)
                mission.addPausedDrone(droneId, (DroneWrapper) drone.clone())
                //pausedDronesData.add(droneMan.getDrone(droneId))

                cmdHandler.cancel(droneId, true)
                withdrawDroneFromMission(droneId, missionId)

                mission.removeActiveDrone(droneId)
            }

            log.info("Mission \"$missionId\" paused")
        }
        else {
            // check if drones are being reassigned
            Thread reassignJob = reassignJobs.get(missionId)
            if (reassignJob != null && reassignJob.isAlive()) {
                // interrupt reassign thread
                reassignJob.interrupt()

                // cancel command for all drones being reassigned
                for (String droneId : mission.pausedDrones.keySet()) {
                    DroneWrapper drone = mission.pausedDrones.get(droneId)
                    cmdHandler.cancel(droneId, true)
                    withdrawDroneFromMission(droneId, missionId)
                }

                log.info("Mission \"$missionId\" reassign job interrupted")
            }
        }

        return pausedDronesData
    }

    synchronized boolean resumeMission(String missionId, MissionResumeAction resumeAction) throws NotFoundException {
        if (isMissionPaused(missionId)) {
            Mission mission = getMission(missionId)

            // TODO: create a custom thread class that extends Thread and implements the reassign job
            Thread reassign = new Thread(new Runnable() {
                @Override
                void run() {
                    // reassign all active drones
                    for (String droneId : mission.pausedDrones.keySet()) {
                        log.info("Reassigning drone $droneId to mission $missionId")
                        DroneWrapper drone = mission.pausedDrones.get(droneId)

                        droneMan.assignToMission(droneId, missionId)
                        try {
                            wrapperMan.reassignDrone(drone, droneId, missionId, resumeAction)
                        } catch (InterruptedException ignored) {
                            log.info("Reassigning drone $droneId to mission $missionId interrupted")
                            return
                        }

                        mission.removePausedDrone(droneId)
                        mission.addActiveDrone(droneId)
                    }

                    // return execution to mission thread after all drones are reassigned
                    threadExecutor.resumeMissionThreads(missionId)
                    mission.status = MissionStatus.RUNNING
                    log.info("Mission \"$missionId\" resumed")
                }
            })
            
            reassign.start()
            reassignJobs.put(missionId, reassign)

            return true
        }
        return false
    }

    synchronized replaceDrone(String missionId, String droneId, String newDroneId) {
        if (isMissionRunning(missionId) && canReplaceDrone(droneId)) {
            DroneWrapper drone = wrapperMan.getDroneWrapper(droneId)
            wrapperMan.replaceDrone(drone, newDroneId, missionId)
            log.info("Mission \"$missionId\": replaced drone \"$droneId\" with drone \"$newDroneId\"")
        }
    }

    synchronized boolean isMissionRunning(String missionId) {
        if (!missions.containsKey(missionId))
            throw new NotFoundException("Mission id not found.")
        MissionStatus status = missions.get(missionId).status
        status == MissionStatus.RUNNING || status == MissionStatus.PENDING
    }

    synchronized boolean isMissionPaused(String missionId) {
        if (!missions.containsKey(missionId))
            throw new NotFoundException("Mission id not found.")
        missions.get(missionId).status == MissionStatus.PAUSED
    }

    def revokeDroneFromMission(String droneId) {
        revokeDroneFromMission(droneId, thisMissionId)
    }

    synchronized revokeDroneFromMission(String droneId, String missionId) {
        if ((isMissionRunning(missionId) || isMissionPaused(missionId)) && getMission(missionId).activeDrones.contains(droneId)) {
            Mission mission = getMission(missionId).removeActiveDrone(droneId)
            if (mission.getRequiredDrones().keySet().contains(droneId))
                mission.removeRequiredDrone(droneId)
            wrapperMan.revokeDrone(droneId)
            mission.plugins.each { pluginId -> pluginMan.disablePluginForDrone(missionId, pluginId, droneId) }
        } else {
            throw new IllegalArgumentException("revoke")
        }
        droneMan.clearDroneMission(droneId)
    }

    synchronized withdrawDroneFromMission(String droneId, String missionId) {
        if (isMissionPaused(missionId) && getMission(missionId).pausedDrones.containsKey(droneId)) {
            Mission mission = getMission(missionId)
            mission.removeActiveDrone(droneId)
            wrapperMan.withdrawDrone(droneId)
        } else {
            throw new IllegalArgumentException("withdraw")
        }
        droneMan.clearDroneMission(droneId)
    }

    boolean isDroneInMission(String droneId) {
        missions.find { entry -> entry.value.activeDrones.contains(droneId) } != null
    }

    Mission getDroneMission(String droneId) {
        missions.find { entry -> entry.value.activeDrones.contains(droneId) }.value
    }

    String getDroneMissionId(String droneId) {
        getDroneMission(droneId).missionId
    }

    boolean isDroneRequired(String missionId, String droneId) {
        missions.get(missionId).requiredDrones.containsKey(droneId)
    }

    boolean canReplaceDrone(String droneId) {
        if (!isDroneInMission(droneId))
            return false
        if (isDroneRequired(getDroneMissionId(droneId), droneId))
            return false
        return true
    }

    Set<String> getDroneMissionRequirements(String missionId, String droneId) {
        missions.get(missionId).requiredDrones.get(droneId)
    }

    private RemoteTaskDTO getDroneRemoteTask(String droneId) {
        Mission mission = getDroneMission(droneId)
        List<RemoteTaskDTO> tasks =  mission.remoteTasks.findAll { task -> task.droneId == droneId }
        return tasks.isEmpty() ? null : tasks.last()
    }

    synchronized def addDroneRemoteTask(RemoteTaskDTO remoteTaskDTO) {
        Mission mission = getDroneMission(remoteTaskDTO.droneId)
        mission.addRemoteTask(remoteTaskDTO)
    }

    def updateDroneRemoteTaskProgress(String droneId, Float progress) {
        RemoteTaskDTO task = getDroneRemoteTask(droneId)
        // A progress message may be received before the remote task has finished initializing, which can cause the
        // progress to be updated before the task is registered to the mission
        if (task != null)
            task.progress = progress
    }

    def updateDroneRemoteTaskState(String droneId, RemoteTaskState state) {
        getDroneRemoteTask(droneId).state = state
    }

    def concludeDroneRemoteTask(String droneId) {
        RemoteTaskDTO task = getDroneRemoteTask(droneId)
        task.state = RemoteTaskState.stopped
        task.end = Instant.now().toEpochMilli()
    }
}
