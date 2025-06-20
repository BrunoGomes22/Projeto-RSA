package org.nap.fleetman.server.mission.dsl

import org.nap.fleetman.server.concurrent.SyncChannel
import org.nap.fleetman.server.model.drone.CommandEnum
import org.nap.fleetman.server.model.drone.DroneState
import org.nap.fleetman.server.model.dsl.*
import org.nap.fleetman.server.remote.RemoteTaskType

class MissionBinding extends Binding {
    private Map variables
    private SyncChannel syncChannel

    MissionBinding(Map vars, SyncChannel syncChannel) {
        this.syncChannel = syncChannel
        this.variables = [
                *        : vars,
                *        : Requirement.values().collectEntries { [(it.name()): it] },
                *        : Direction.values().collectEntries { [(it.name()): it] },
                *        : TimeUnit.values().collectEntries { [(it.name()): it, (it.name() + 's'): it, (it.abbreviation): it] },
                *        : DistanceUnit.values().collectEntries { [(it.name()): it, (it.name() + 's'): it, (it.abbreviation): it] },
                *        : DroneState.values().collectEntries { [(it.name().toLowerCase()): it] },
                *        : CommandEnum.values().collectEntries { [(it.name().toLowerCase()): it] },
                *        : RemoteTaskType.values().collectEntries { [(it.name()): it] },
                move     : CommandEnum.go_to,
                home     : CommandEnum.return_to_launch,
                Direction: Direction,
                Distance : Distance,
                Speed    : Speed,
                Time     : Time
        ]
    }

    def getVariable(String name) {
        def var = variables[name.toLowerCase()] ?: variables[name] ?: super.getVariable(name)
        if (var instanceof DroneWrapper) {
            if (var.revoked)
                return null
            return var
        } else {
            return var
        }
    }

    // FIXME deprecated
    void wait(DroneWrapper dw) {
        syncChannel.waitTelem(dw)
    }
}