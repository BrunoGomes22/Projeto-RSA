package org.nap.fleetman.server.remote


import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.springframework.stereotype.Service

// Client interface for mission scripts to interact with remote tasks executed drone-side
@Service
class RemoteTaskClient {
    private RemoteTaskHandler remoteTaskHandler

    RemoteTaskClient(RemoteTaskHandler remoteTaskHandler) {
        this.remoteTaskHandler = remoteTaskHandler
    }

    def start(RemoteTaskType task) {
        parseVerbs(task, RemoteTaskAction.start)
    }

    def stop(RemoteTaskType task) {
        parseVerbs(task, RemoteTaskAction.stop)
    }

    def pause(RemoteTaskType task) {
        parseVerbs(task, RemoteTaskAction.pause)
    }

    def resume(RemoteTaskType task) {
        parseVerbs(task, RemoteTaskAction.resume)
    }

    // Parse provided parameters and drone with the "using" and "with" keywords though the usage of maps + closures
    private parseVerbs(RemoteTaskType task, RemoteTaskAction action) {
        [using: { DroneWrapper drone ->
            RemoteTask remoteTask = remoteTaskHandler.processTaskRequest([_wrapper: drone], drone.id, task, action)
            return remoteTask.localThreadId
        },
         with : { Map params ->
             [using: { DroneWrapper drone ->
                 RemoteTask remoteTask = remoteTaskHandler.processTaskRequest(params + [_wrapper: drone], drone.id, task, action)
                 return remoteTask.localThreadId
             }
             ]
         }]
    }
}
