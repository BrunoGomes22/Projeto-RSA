package org.nap.fleetman.server.plugins

import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.model.dsl.Time
import org.nap.fleetman.server.model.dsl.TimeUnit

import java.util.concurrent.ConcurrentHashMap

class ScheduledPlugin extends Plugin {
    private Closure callback
    private Map<String, List<DroneWrapper>> missionDrones

    ScheduledPlugin(PluginConfig cfg, PluginBinding binding) {
        super(cfg, binding)
        callback = cfg.callback
        missionDrones = new ConcurrentHashMap<>()
    }

    def monitor(List<DroneWrapper> drones, String missionId) {
        missionDrones[missionId] = drones
        callback.getProperty('taskLauncher').run({
            while (true) {
                Thread.sleep(getRate(missionId))
                callback.call(missionDrones[missionId].collect())
            }
        }, missionId: missionId, pluginId: id)
    }

    Long getRate(String missionId) {
        def rate = this.getCurrentVariables(missionId).rate
        if (rate == null)
            rate = 1000
        else if (rate instanceof Time)
            rate = rate.convertTo(TimeUnit.millisecond)
        if (rate < 1)
            rate = 1000

        return rate
    }

    def addDrone(DroneWrapper drone, String missionId) {
        missionDrones[missionId].add(drone)
    }

    def removeDrone(DroneWrapper drone, String missionId) {
        missionDrones[missionId].remove(drone)
    }

    def removeDrone(String droneId, String missionId) {
        DroneWrapper drone = missionDrones[missionId].find { drone -> drone.id == droneId}
        removeDrone(drone, missionId)
    }


    boolean disableForMission(String missionId) {
        boolean exists = super.disableForMission(missionId)
        if (exists)
            missionDrones.remove(missionId)
        return exists
    }
}
