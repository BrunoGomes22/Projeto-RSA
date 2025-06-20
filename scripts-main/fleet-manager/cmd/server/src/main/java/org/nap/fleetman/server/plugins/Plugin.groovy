package org.nap.fleetman.server.plugins

import java.util.concurrent.ConcurrentLinkedQueue

abstract class Plugin {
    private PluginConfig cfg
    private PluginBinding binding
    Queue<String> missionIds
    File file

    Plugin(PluginConfig cfg, PluginBinding binding) {
        this.cfg = cfg
        this.binding = binding
        binding.setPluginVariableNames(cfg.vars.keySet() + cfg.input.keySet())
        missionIds = new ConcurrentLinkedQueue<>()
    }

    def enableForMission(String missionId, Map input) {
        PluginValidator.validateMissionInput(cfg.input, input)
        binding.setMissionVariables(missionId, cfg.getVariables(input))
        missionIds.add(missionId)
        if (cfg.init != null)
            cfg.init.call()
    }

    boolean disableForMission(String missionId) {
        if (missionIds.remove(missionId)) {
            binding.removeMissionVariables(missionId)
            return true
        }
        return false
    }

    def getCurrentVariables(String missionId) {
        binding.getMissionVariables(missionId)
    }

    def getType() {
        cfg.type
    }

    def getId() {
        cfg.id
    }

    def getInput() {
        cfg.input
    }
}
