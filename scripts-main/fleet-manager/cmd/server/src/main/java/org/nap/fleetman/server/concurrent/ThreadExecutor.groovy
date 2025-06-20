package org.nap.fleetman.server.concurrent

import groovyx.gpars.dataflow.Dataflows
import org.nap.fleetman.server.utils.CommandHandler
import org.springframework.beans.factory.annotation.Value
import org.springframework.stereotype.Service

import java.util.concurrent.BlockingQueue
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit

@Service
class ThreadExecutor {
    private CommandHandler cmdHandler
    private Queue<MissionThread> availableWorkers
    private BlockingQueue<Map> queue
    private ConcurrentHashMap<String, ConcurrentLinkedQueue<MissionThread>> missionThreads
    private ConcurrentHashMap<String, MissionThread> allRunningThreads
    private Dataflows syncLaunchedThreads
    @Value('${fleetman.thread.keepAliveTime:60}')
    private int keepAliveTime

    ThreadExecutor(CommandHandler cmdHandler) {
        this.cmdHandler = cmdHandler
        availableWorkers = new ConcurrentLinkedQueue<>()
        queue = new LinkedBlockingQueue<>()
        missionThreads = new ConcurrentHashMap<>()
        allRunningThreads = new ConcurrentHashMap<>()
        syncLaunchedThreads = new Dataflows()
    }

    void execute(Runnable task, String taskId, String missionId, String pluginId, String droneId) {
        if (availableWorkers.isEmpty()) {
            new MissionThread().start()
        }
        queue.add([task: task, taskId: taskId, missionId: missionId, pluginId: pluginId, droneId: droneId])
        syncLaunchedThreads.getProperty(taskId)
        syncLaunchedThreads.remove(taskId)
    }

    synchronized stopThread(String taskId) {
        MissionThread thread = allRunningThreads.get(taskId)
        if (thread == null)
            return
        def droneId = thread.currentCommandDroneId
        thread.interrupt()
        // TODO move somewhere else
        if (droneId != null)
            cmdHandler.cancel(droneId, true)
    }

    synchronized stopMissionThreadsExceptThis(String missionId) {
        def threads = missionThreads.get(missionId)
        threads.each {thread ->
            if (thread != Thread.currentThread())
                thread.interrupt()
        }
    }

    synchronized pauseMissionThreads(String missionId) {
        missionThreads[missionId].findAll{ thread ->
            thread.missionId == missionId }.each { thread ->
                thread.suspend()
            }
    }

    synchronized resumeMissionThreads(String missionId) {
        missionThreads[missionId].findAll{ thread ->
            thread.missionId == missionId }.each { thread ->
                thread.resume()
            }
    }

    synchronized stopMissionThreads(String missionId) {
        stopMissionThreadsExceptThis(missionId)
        Thread.currentThread().interrupt()
    }

    void stopMissionPluginThreads(String missionId, String pluginId) {
        missionThreads[missionId].findAll{ thread ->
            thread.pluginId == pluginId }.each { thread ->
                thread.interrupt()
        }
    }

    def stopDronePluginThread(String missionId, String droneId, String pluginId) {
        missionThreads[missionId].findAll{ thread ->
            thread.pluginId == pluginId && thread.droneId == droneId }.each { thread ->
                thread.interrupt()
            Thread.currentThread()
        }
    }

    boolean missionHasOtherThreadsRunning(String missionId) {
        missionThreads.containsKey(missionId) && missionThreads[missionId].size() > 1
    }

    String getMissionIdFromTask(String taskId) {
        MissionThread thread = allRunningThreads.get(taskId)
        thread ? thread.missionId : null
    }

    class MissionThread extends Thread {
        private static int id = 0
        // Current task ID
        String taskId
        // ID of the mission which launched this thread
        String missionId
        // If this thread was launched by a plugin, indicates its ID
        String pluginId
        // If this thread was launched by a monitor plugin, indicates the ID of the monitored drone
        // This is the current meaning but it could be expanded for other uses in the future with caution
        String droneId
        // If a command is currently being executed in this thread, indicates the corresponding drone ID
        String currentCommandDroneId

        MissionThread() {
            super("mission-thread-${id++}")
        }

        void run() {
            while (true) {
                Map data
                try {
                    data = queue.poll(keepAliveTime, TimeUnit.SECONDS)
                } catch (InterruptedException ignored) {}

                availableWorkers.remove(this)
                if (data == null)
                    break

                setIds(data.taskId, data.missionId, data.pluginId, data.droneId)
                registerThreadTask()
                try {
                    data.task.run()
                } catch (InterruptedException ignored) {}
                concludeThreadTask()
                resetIds()
                availableWorkers.add(this)
            }
        }

        private void registerThreadTask() {
            if (missionThreads.containsKey(missionId))
                missionThreads[missionId].add(this)
            else {
                def threadList = new ConcurrentLinkedQueue()
                threadList.add(this)
                missionThreads.put(missionId, threadList)
            }
            allRunningThreads.put(taskId, this)
            syncLaunchedThreads.setProperty(taskId, true)
        }

        synchronized private void concludeThreadTask() {
            missionThreads[missionId].remove(this)
            synchronized (missionThreads[missionId]) {
                if (missionThreads[missionId] != null && missionThreads[missionId].isEmpty())
                    missionThreads.remove(missionId)
            }
            allRunningThreads.remove(taskId)
        }

        private void setIds(String taskId, String missionId, String pluginId, String droneId) {
            this.taskId = taskId
            this.missionId = missionId
            this.pluginId = pluginId
            this.droneId = droneId
            this.currentCommandDroneId = null
        }

        private void resetIds() {
            taskId = null
            missionId = null
            pluginId = null
            droneId = null
        }

        @Override
        String toString(){
            super.toString() + " Task: $taskId Mission: $missionId Plugin: $pluginId Drone: $droneId Current command drone: $currentCommandDroneId"
        }
    }
}
