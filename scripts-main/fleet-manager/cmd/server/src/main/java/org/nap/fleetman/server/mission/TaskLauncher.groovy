package org.nap.fleetman.server.mission

import groovyx.gpars.dataflow.Dataflows
import org.nap.fleetman.server.concurrent.ThreadExecutor
import org.nap.fleetman.server.exceptions.TaskException
import org.nap.fleetman.server.mission.dsl.*
import org.nap.fleetman.server.plugins.PluginManager
import org.nap.fleetman.server.exceptions.ExceptionHandler
import org.nap.fleetman.server.utils.MissionUtils
import org.springframework.beans.factory.annotation.Value
import org.springframework.stereotype.Service

import javax.annotation.PostConstruct

@Service
class TaskLauncher {
    private Dataflows channel
    private MissionManager missionMan
    private ThreadExecutor threadExecutor
    private ExceptionHandler exceptionHandler
    private List<String> runningTasks
    @Value('${fleetman.mission.printStackTrace:true}')
    private boolean printFullStackTrace

    TaskLauncher(MissionManager missionMan, ThreadExecutor threadExecutor) {
        this.channel = new Dataflows()
        this.missionMan = missionMan
        this.threadExecutor = threadExecutor
        runningTasks = new ArrayList<>()
    }

    @PostConstruct
    private initExceptionHandler() {
        this.exceptionHandler = new ExceptionHandler(printFullStackTrace)
    }

    def run(Map args = [:], Closure... routines) {
        def taskIds = []
        String missionId = args.missionId ?: MissionManager.thisMissionId
        String pluginId = args.pluginId ?: PluginManager.isPluginThread() ? PluginManager.thisPluginId : null
        String droneId = args.droneId ?: MissionManager.isDroneThread() ? MissionManager.thisThreadDroneId : null

        routines.each { Closure c ->
            // Generate task id
            String taskId = MissionUtils.generateTaskId()
            taskIds.add(taskId)
            // Add to running tasks list
            runningTasks.add(taskId)
            // Launch task
            submitTask({
                c.run()
                // Reached the end of the task
                handleFinishTask(taskId, true)
            }, taskId, missionId, pluginId, droneId)
        }
        // If only one task was sent, return it instead of a list
        taskIds.size() > 1 ? taskIds : taskIds.get(0)
    }

    void submitTask(Closure routine, String taskId, String missionId, String pluginId, String droneId) {
        Runnable task = { ->
            use(DistanceCategory, TimeCategory, DegreeCategory, UtilCategory, SpeedCategory, PositionCategory) {
                def result = exceptionHandler.runAndCatchExceptions(routine, pluginId != null)
                if (!result.isEmpty())
                    missionMan.concludeFailedMission(result.endMsg, result.logMsg, pluginId)
            }
        }
        threadExecutor.execute(task, taskId, missionId, pluginId, droneId)
    }

    synchronized private handleFinishTask(String taskId, boolean concluded) {
        // Remove task from list of running threads
        runningTasks.remove(taskId)
        // Wake threads waiting for this task to be finished
        channel.setProperty(taskId, concluded)
    }

    def stop(List<String> taskIds) {
        // Check if all the given taskIds have been registered
        if (!runningTasks.containsAll(taskIds))
            throw new TaskException(TaskException.State.STOP)

        taskIds.each { id ->
            // Stop the task thread
            threadExecutor.stopThread(id)
            handleFinishTask(id, false)
        }
    }

    def stop(String... taskIds) {
        stop(taskIds.toList())
    }

    private boolean isTaskIdWaitable(String taskId, String missionId) {
        String threadExecMissionId = threadExecutor.getMissionIdFromTask(taskId)
        // If no thread with this taskId is running, we can wait for this task if it has already been completed
        if (threadExecMissionId == null)
            return finished(taskId) && channel.contains(taskId)
        // If there is a thread with this taskId, we can wait if it is from the same mission and is marked as running
        return threadExecMissionId == missionId && running(taskId)
    }

    def wait(String taskId, String missionId=null) {
        if (taskId == null)
            throw new TaskException(TaskException.State.NULL)
        if (!isTaskIdWaitable(taskId, missionId ?: MissionManager.thisMissionId))
            throw new TaskException(TaskException.State.WAIT)
        return channel.getProperty(taskId)
    }

    def running(String taskId) {
        return runningTasks.contains(taskId)
    }

    def finished(String taskId) {
        return !runningTasks.contains(taskId)
    }
}
