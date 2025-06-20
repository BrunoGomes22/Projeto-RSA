package org.nap.fleetman.server.remote

import org.nap.fleetman.server.concurrent.SyncChannel
import org.nap.fleetman.server.core.DroneManager
import org.nap.fleetman.server.exceptions.MissionException
import org.nap.fleetman.server.mission.MissionManager
import org.nap.fleetman.server.mission.TaskLauncher
import org.nap.fleetman.server.model.mission.MissionEndMsg
import org.nap.fleetman.server.utils.DroneLog
import org.ros2.rcljava.RCLJava
import org.ros2.rcljava.client.Client
import org.ros2.rcljava.node.Node
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Service

import java.util.concurrent.ConcurrentHashMap

// Sends requests regarding remotely executed tasks and keeps track of the status of these tasks
@Service
class RemoteTaskHandler {
    private static final Logger log = LoggerFactory.getLogger(RemoteTaskHandler.class)
    private MissionManager missionMan
    private DroneManager droneMan
    private TaskLauncher taskLauncher
    private SyncChannel syncChannel
    // ROS2 node responsible for sending service requests
    private Node node
    // Remote task status
    private Map<String, RemoteTask> droneRemoteTasks

    RemoteTaskHandler(MissionManager missionMan, DroneManager droneMan, TaskLauncher taskLauncher, SyncChannel syncChannel) {
        this.missionMan = missionMan
        this.droneMan = droneMan
        this.taskLauncher = taskLauncher
        this.syncChannel = syncChannel
        node = RCLJava.createNode("ground_task_handler")
        droneRemoteTasks = new ConcurrentHashMap<>()
    }

    // This reference is created as soon as a drone wrapper is instantiated, even if it does start any remote task while
    // in mission; this is done to avoid creating a dependency to the WrapperManager - the drone wrapper object contains
    // a reference to its possible remote task as soon as it is assigned without being updated.
    RemoteTask createDroneRemoteTaskReference(String droneId) {
        RemoteTask task = new RemoteTask()
        droneRemoteTasks[droneId] = task
        return task
    }

    // Process received task requests
    RemoteTask processTaskRequest(Map params = null, String droneId, RemoteTaskType task, RemoteTaskAction action) {
        def request = RemoteTaskRequestBuilder.createTaskRequest(params, task, action)
        if (validateRemoteAction(droneId, task, action)) {
            def result = sendClientRequest(droneId, task, request)
            if (!result.success)
                throw new MissionException(MissionEndMsg.REMOTE_TASK, "Failed to $action task '$task' for drone '$droneId' - $result.message")
            if (action == RemoteTaskAction.start)
                startRemoteTask(task, droneId)
            else if (action == RemoteTaskAction.resume)
                resumeRemoteTask(droneId)
        }
        // Returns the RemoteTask object
        return droneRemoteTasks[droneId]
    }

    // Validate if the provided request can be made at the moment
    private boolean validateRemoteAction(String droneId, RemoteTaskType task, RemoteTaskAction action) {
        switch (action) {
            case RemoteTaskAction.start:
                if (droneRemoteTasks[droneId].taskType == null)
                    return true
                if (droneRemoteTasks[droneId].taskType != task)
                    throw new MissionException(MissionEndMsg.REMOTE_TASK,
                            "Attempted to start task '$task' for drone '$droneId' while already running task '${droneRemoteTasks[droneId].taskType}'")
                log.warn("Attempted to start task '$task' for drone '$droneId' which was already started")
                return false
            case RemoteTaskAction.stop:
                if (droneRemoteTasks[droneId].taskType == task)
                    return true
                throw new MissionException(MissionEndMsg.REMOTE_TASK, "Attempted to stop task '$task' for drone '$droneId' which was not running")
            case RemoteTaskAction.pause:
                if (droneRemoteTasks[droneId].taskType != task)
                    throw new MissionException(MissionEndMsg.REMOTE_TASK,"Attempted to pause task '$task' for drone '$droneId' which was not running")
                if (droneRemoteTasks[droneId].running)
                    return true
                log.warn("Attempted to pause task '$task' for drone '$droneId' which was already paused")
                return false
            case RemoteTaskAction.resume:
                if (droneRemoteTasks[droneId].taskType != task)
                    throw new MissionException(MissionEndMsg.REMOTE_TASK,
                            "Attempted to resume task '$task' for drone '$droneId' which was not running'")
                if (!droneRemoteTasks[droneId].running)
                    return true
                log.warn("Attempted to resume task '$task' for drone '$droneId' which was already running")
                return false
        }
    }

    // Sends the request to the corresponding ROS service
    private sendClientRequest(String droneId, RemoteTaskType task, request) {
        Client client = RemoteTaskRequestBuilder.createTaskClient(droneId, task, node)
        if (!client.isServiceAvailable())
            throw new MissionException(MissionEndMsg.REMOTE_TASK, "No service task '$task' available for drone '$droneId'")
        client.waitForService()
        return client.asyncSendRequest(request).get()
    }

    // Initialize remote task tracking locally
    private startRemoteTask(RemoteTaskType task, String droneId) {
        // Launch thread which will wait for completion and return task ID
        String threadId = taskLauncher.run([droneId: droneId], {syncChannel.waitRemoteTask(droneId, task.toString())})
        // Initialize task parameters
        droneRemoteTasks[droneId].initTask(task, threadId)
        // Update mission and drone data to include this task
        missionMan.addDroneRemoteTask(new RemoteTaskDTO(task, droneId))
        droneMan.setDroneRemoteTask(droneId, task)
    }

    // Update task's running state
    private resumeRemoteTask(String droneId) {
        droneRemoteTasks[droneId].running = true
        missionMan.updateDroneRemoteTaskState(droneId, RemoteTaskState.running)
    }

    // Check whether a task is registered as being requested to run for a given drone
    private boolean taskIsNotRegistered(String droneId, RemoteTaskType task) {
        return droneRemoteTasks[droneId] == null || droneRemoteTasks[droneId].taskType != task
    }

    // Update remote task status after receiving a mission status message
    def updateRemoteTaskStatus(drone_interfaces.msg.MissionStatus msg) {
        RemoteTaskType taskType = msg.task.toLowerCase() as RemoteTaskType
        RemoteStatusMessage msgType = msg.type.toLowerCase() as RemoteStatusMessage
        String droneId = msg.droneId
        switch (msgType) {
            case RemoteStatusMessage.progress:
                // Only process message if this task was launched through the ground station
                if (taskIsNotRegistered(droneId, taskType))
                    break
                droneRemoteTasks[droneId].progress = msg.message as Float
                missionMan.updateDroneRemoteTaskProgress(droneId, msg.message as Float)
                break
            case RemoteStatusMessage.state:
                DroneLog.info(droneId, "Remote task '$taskType' state transition - $msg.message")
                RemoteTaskState state = msg.message.toLowerCase() as RemoteTaskState
                // If the service is in emergency mode, attempt to switch it to normal mode even if no task is running
                if (state == RemoteTaskState.emergency) {
                    if (!taskIsNotRegistered(droneId, taskType)) {
                        concludeRemoteTask(droneId, taskType, false)
                        missionMan.concludeFailedMission(missionMan.getDroneMissionId(droneId), MissionEndMsg.REMOTE_TASK,
                                "Remote task '$taskType' for drone '$droneId' entered emergency mode")
                    }
                    def request = RemoteTaskRequestBuilder.createTaskRequest(taskType, RemoteTaskAction.normal)
                    def result = sendClientRequest(droneId, taskType, request)
                    if (!result.success)
                        DroneLog.warn(droneId, "Failed to reset emergency state of task '$taskType' - $result.message")
                    else
                        DroneLog.info(droneId, "Reset emergency state of task '$taskType'")
                // Ignore message if the task is not started by the ground station
                } else if (taskIsNotRegistered(droneId, taskType))
                    break
                else if (state == RemoteTaskState.paused) {
                    droneRemoteTasks[droneId].running = false
                    missionMan.updateDroneRemoteTaskState(droneId, RemoteTaskState.paused)
                } else if (state == RemoteTaskState.stopped)
                    concludeRemoteTask(droneId, taskType, true)
                break
            case RemoteStatusMessage.error:
                missionMan.concludeFailedMission(missionMan.getDroneMissionId(droneId), MissionEndMsg.REMOTE_TASK,
                        "Error on remote task '$taskType' for drone '$droneId' - $msg.message")
                break
            case RemoteStatusMessage.info:
                DroneLog.info(droneId, msg.message)
                break
            case RemoteStatusMessage.planning:
                // TODO not implemented yet
                break
            case RemoteStatusMessage.following:
                // TODO not implemented yet
                break
        }
    }

    // Cancel drone task, requested internally when the mission stops or the drone is removed from the mission
    def stopAndRemoveRemoteDroneTask(String droneId) {
        RemoteTask task = droneRemoteTasks[droneId]
        if (task.taskType != null) {
            try {
                processTaskRequest(droneId, task.taskType, RemoteTaskAction.stop)
            } catch (MissionException e) {
                log.warn("Unable to request remote task to stop on mission conclusion - $e.message")
                concludeRemoteTask(droneId, task.taskType, false)
            }
        }
        droneRemoteTasks.remove(droneId)
    }

    // Clean up after finishing the remote task
    private concludeRemoteTask(String droneId, RemoteTaskType task, boolean success) {
        droneRemoteTasks[droneId].terminateTask()
        String missionId = missionMan.getDroneMission(droneId).missionId
        syncChannel.unlockRemoteTask(droneId, task.toString(), success)
        missionMan.concludeDroneRemoteTask(droneId)
        droneMan.setDroneRemoteTask(droneId, null)
        log.info("Mission '$missionId' - Remote task '$task' for drone '$droneId' concluded")
    }
}
