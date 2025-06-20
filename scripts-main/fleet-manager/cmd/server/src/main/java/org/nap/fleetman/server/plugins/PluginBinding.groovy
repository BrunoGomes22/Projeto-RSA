package org.nap.fleetman.server.plugins

import org.nap.fleetman.server.concurrent.SyncChannel
import org.nap.fleetman.server.mission.MissionManager
import org.nap.fleetman.server.mission.dsl.MissionBinding

import java.util.concurrent.ConcurrentHashMap

class PluginBinding extends MissionBinding {
    // Set of variables declared by the plugin
    private Set<String> pluginVariableNames
    // Mapping of the plugin variables to the current value in each mission which is using the plugin
    private Map<String, Object> missionVariables

    PluginBinding(Map vars, SyncChannel syncChannel) {
        super(vars, syncChannel)
        pluginVariableNames = new HashSet<>()
        missionVariables = new ConcurrentHashMap<>()
    }

    def setPluginVariableNames(Set<String> pluginVariableNames) {
        this.pluginVariableNames = pluginVariableNames
    }

    def getVariable(String name) {
        if (pluginVariableNames.contains(name))
            missionVariables.get(MissionManager.thisMissionId + name)
        else
            super.getVariable(name)
    }

    void setVariable(String name, Object value) {
        if (pluginVariableNames.contains(name))
            missionVariables.put(MissionManager.thisMissionId + name, value)
        else
            super.setVariable(name, value)
    }

    void setMissionVariables(String missionId, Map<String, Object> vars) {
        vars.each {var, value -> missionVariables.put(missionId + var, value)}
    }

    Map getMissionVariables(String missionId) {
        missionVariables.findAll {var, _ -> var.startsWith(missionId)}.collectEntries {var, value -> [(var.split(missionId)[1]):value]}
    }

    void removeMissionVariables(String missionId) {
        pluginVariableNames.each {var -> missionVariables.remove(missionId + var)}
    }
}
