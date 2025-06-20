package org.nap.fleetman.server.mission.dsl

import groovyx.gpars.dataflow.Dataflows
import org.nap.fleetman.server.mission.*
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.model.dsl.Time
import org.nap.fleetman.server.model.dsl.TimeUnit
import org.nap.fleetman.server.plugins.PluginClient
import org.nap.fleetman.server.remote.RemoteTaskClient

abstract class MissionHandlerBaseScript extends Script {
    @Delegate
    @Lazy
    CommandParser cmdParser = this.binding.cmdparser
    @Delegate
    @Lazy
    DroneAssigner assigner = this.binding.assigner
    @Delegate
    @Lazy
    PluginClient pluginClient = this.binding.pluginClient
    @Delegate
    @Lazy
    TaskLauncher taskLauncher = this.binding.taskLauncher
    @Delegate
    @Lazy
    RemoteTaskClient remoteTaskClient = this.binding.remoteTaskClient
    @Delegate
    @Lazy
    MessageClient msgClient = this.binding.msgClient
    @Delegate
    @Lazy
    MissionClient missionClient = this.binding.missionClient
    @Delegate
    @Lazy
    Dataflows channel = this.binding.channel
    @Delegate
    @Lazy
    PositionUtils posUtils

    static boolean not(boolean val) {
        return !val
    }

    static long now() {
        new Date().getTime()
    }

    // FIXME deprecated
    void wait(DroneWrapper dw) {
        this.binding.wait(dw)
    }

    // FIXME
    Object max(Object o1, Object o2) {
        o1 > o2 ? o1 : o2
    }

    // FIXME
    Object min(Object o1, Object o2) {
        o1 < o2 ? o1 : o2
    }
}