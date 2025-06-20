package org.nap.fleetman.server.plugins

import org.nap.fleetman.server.model.dsl.DroneWrapper

class MonitorPlugin extends Plugin {
    private Closure callback

    MonitorPlugin(PluginConfig cfg, PluginBinding binding) {
        super(cfg, binding)
        callback = cfg.callback
    }

    def monitorDrone(DroneWrapper drone, String missionId) {
        callback.getProperty('taskLauncher').run({
            while (!drone.revoked) {
                callback.call(drone)
                callback.wait(drone)
            }
        }, missionId: missionId, pluginId: id, droneId: drone.id)
    }
}
