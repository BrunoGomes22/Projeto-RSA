package org.nap.fleetman.server.plugins

import org.nap.fleetman.server.mission.dsl.DegreeCategory
import org.nap.fleetman.server.mission.dsl.DistanceCategory
import org.nap.fleetman.server.mission.dsl.MissionHandlerBaseScript
import org.nap.fleetman.server.mission.dsl.PositionCategory
import org.nap.fleetman.server.mission.dsl.SpeedCategory
import org.nap.fleetman.server.mission.dsl.TimeCategory
import org.nap.fleetman.server.mission.dsl.UtilCategory

abstract class PluginConfigReaderScript extends MissionHandlerBaseScript {
    PluginConfig plugin

    void plugin(@DelegatesTo(PluginConfig) Closure config) {
        use(DistanceCategory, TimeCategory, DegreeCategory, UtilCategory, SpeedCategory, PositionCategory) {
            if (plugin != null)
                println "two cfgs one file"
            plugin = new PluginConfig()
            plugin.with config
        }
    }

}