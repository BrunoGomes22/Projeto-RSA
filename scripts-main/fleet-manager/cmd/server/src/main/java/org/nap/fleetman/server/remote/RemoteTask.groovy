package org.nap.fleetman.server.remote

// Remote task data
class RemoteTask {
    RemoteTaskType taskType
    // Id of the thread that is locally waiting for this task's conclusion
    String localThreadId
    // Current progress of the task from 0-100
    Number progress
    // If this task is currently running (execution can be paused)
    Boolean running

    def initTask(RemoteTaskType taskType, String localThreadId) {
        this.taskType = taskType
        this.localThreadId = localThreadId
        progress = 0
        running = true
    }

    def terminateTask() {
        taskType = null
        localThreadId = null
        progress = null
        running = null
    }
}
