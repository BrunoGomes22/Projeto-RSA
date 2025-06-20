package org.nap.fleetman.server.utils

import groovy.json.JsonBuilder
import org.nap.fleetman.server.core.MessagePublisher
import org.nap.fleetman.server.concurrent.SyncChannel
import org.nap.fleetman.server.model.drone.CommandEnum
import org.springframework.stereotype.Service

import org.slf4j.Logger
import org.slf4j.LoggerFactory

@Service
class CommandHandler {
    private MessagePublisher pub
    private SyncChannel syncChannel
    private static final Logger log = LoggerFactory.getLogger(CommandHandler.class)

    CommandHandler(MessagePublisher pub, SyncChannel syncChannel) {
        this.pub = pub
        this.syncChannel = syncChannel
    }

    private sendCommand(Map cmd, boolean wait) {
        pub.publishCommand(new JsonBuilder(cmd).toString())
        if (wait)
            return waitForCmdResult(cmd)
        else
            return cmd
    }

    boolean waitForCmdResult(Map cmd) {
        syncChannel.waitCommand(cmd.droneId, cmd.cmd)
    }

    def arm(String droneId, boolean wait=false) {
        sendCommand(droneId: droneId,
                mode: 'action',
                cmd: CommandEnum.arm.toString(), wait)
    }

    def disarm(String droneId, boolean wait=false) {
        sendCommand(droneId: droneId,
                mode: 'action',
                cmd: CommandEnum.disarm.toString(), wait)
    }

    def takeoff(String droneId, Number alt, boolean wait=false) {
        def cmd = [droneId: droneId,
                   mode   : 'action',
                   cmd    : CommandEnum.takeoff.toString()]
        if (alt != null)
            cmd += [alt: alt]
        sendCommand(cmd, wait)
    }

    private static baseGoToCmd(String droneId, Number lat, Number lon, Number alt, Number speed) {
        def cmd = [droneId: droneId,
                   mode   : 'action',
                   cmd    : CommandEnum.go_to.toString(),
                   lat    : lat,
                   lon    : lon]
        if (alt != null)
            cmd += [alt: alt]
        if (speed != null)
            cmd += [speed: speed]
        return cmd
    }

    def goTo(String droneId, Number lat, Number lon, Number alt, Number speed, boolean wait=false) {
        sendCommand(baseGoToCmd(droneId, lat, lon, alt, speed), wait)
    }

    def goTo(String droneId, Number lat, Number lon, Number alt, Number speed, Number yaw, boolean wait=false) {
        def cmd = baseGoToCmd(droneId, lat, lon, alt, speed)
        cmd += [yaw: yaw]
        sendCommand(cmd, wait)
    }

    private static baseMoveCmd(String droneId, Number x, Number y, Number z, Number speed) {
        def cmd = [droneId: droneId,
                   mode   : 'custom',
                   cmd    : 'move', // TODO in the future this should be in enum
                   x      : x,
                   y      : y,
                   z      : z]
        if (speed != null)
            cmd += [speed: speed]
        
        log.info("Command: $cmd")
        return cmd
    }

    def move(String droneId, Number x, Number y, Number z, Number speed, boolean wait=false) {
        sendCommand(baseMoveCmd(droneId, x, y, z, speed), wait)
    }

    def move(String droneId, Number x, Number y, Number z, Number speed, Number yaw, boolean wait=false) {
        def cmd = baseMoveCmd(droneId, x, y, z, speed)
        cmd += [yaw: yaw]
        sendCommand(cmd, wait)
    }

    def turn(String droneId, Number deg, boolean wait=false) {
        sendCommand(droneId: droneId,
                mode: 'custom',
                cmd: 'turn', // TODO in the future this should be in enum
                deg: deg, wait)
    }

    def land(String droneId, boolean wait=false) {
        sendCommand(droneId: droneId,
                mode: 'action',
                cmd: CommandEnum.land.toString(), wait)
    }

    def returnToLaunch(String droneId, Number alt, Number speed, boolean wait=false) {
        def cmd = [droneId: droneId,
                   mode   : 'action',
                   cmd    : CommandEnum.return_to_launch.toString()]
        if (alt != null)
            cmd += [alt: alt]
        if (speed != null)
            cmd += [speed: speed]
        sendCommand(cmd, wait)
    }

    def cancel(String droneId, boolean wait=false) {
        sendCommand(droneId: droneId,
                mode: 'custom',
                cmd: CommandEnum.cancel.toString(), wait)
    }
}
