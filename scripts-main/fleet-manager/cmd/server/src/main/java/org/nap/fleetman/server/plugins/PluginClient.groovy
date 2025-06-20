package org.nap.fleetman.server.plugins

import org.nap.fleetman.server.mission.MissionManager
import org.nap.fleetman.server.mission.WrapperManager
import org.nap.fleetman.server.model.mission.MissionEndMsg
import org.springframework.stereotype.Service

@Service
class PluginClient {
    private PluginManager pluginMan
    private MissionManager missionMan
    private WrapperManager wrapperMan

    PluginClient(PluginManager pluginMan, MissionManager missionMan, WrapperManager wrapperMan) {
        this.pluginMan = pluginMan
        this.missionMan = missionMan
        this.wrapperMan = wrapperMan
    }

    void enable(Map input, String pluginId) {
        enable(pluginId, input)
    }

    void enable(String pluginId, Map input = [:]) {
        def missionId = MissionManager.thisMissionId
        def wrappers = wrapperMan.getDroneWrappers(missionMan.getMission(missionId).activeDrones)
        if (pluginMan.enablePlugin(pluginId, missionId, wrappers, input))
            missionMan.addPluginToMission(missionId, pluginId)
        else
            missionMan.concludeFailedMission(MissionEndMsg.PLUGIN_MISSING, "Plugin \"${pluginId}\" not found")
    }

    void disable(String pluginId) {
        def missionId = MissionManager.thisMissionId
        def status = pluginMan.disablePlugin(pluginId, missionId)
        if (status < 0)
            missionMan.concludeFailedMission(MissionEndMsg.PLUGIN_MISSING, "Plugin \"${pluginId}\" not found")
        else if (status > 0)
            missionMan.removePluginFromMission(missionId, pluginId)
    }
}
