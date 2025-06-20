package org.nap.fleetman.server.mission

import org.nap.fleetman.server.exceptions.InstructionParsingException
import org.nap.fleetman.server.mission.dsl.DistanceCategory
import org.nap.fleetman.server.mission.dsl.PositionCategory
import org.nap.fleetman.server.model.drone.CommandEnum
import org.nap.fleetman.server.model.dsl.*
import org.nap.fleetman.server.model.telemetry.Position
import org.nap.fleetman.server.utils.CommandHandler
import org.nap.fleetman.server.utils.CommandValidator
import org.springframework.beans.factory.annotation.Value
import org.springframework.stereotype.Service
import org.nap.fleetman.server.model.drone.DroneState

import org.slf4j.Logger
import org.slf4j.LoggerFactory

@Service
class CommandParser {
    private CommandHandler cmdHandler
    @Value('${fleetman.mission.wait:true}')
    private boolean wait
    private static final Logger log = LoggerFactory.getLogger(CommandParser.class)

    CommandParser(CommandHandler cmdHandler) {
        this.cmdHandler = cmdHandler
    }

    def arm(DroneWrapper drone) {
        // only arm if drone not in air
        if (drone.state != DroneState.READYINAIR)
            cmdHandler.arm(drone.id, wait)
        else
            log.warn("Drone ${drone.id} is already in the air. Cannot arm.")
    }

    def disarm(DroneWrapper drone) {
        cmdHandler.disarm(drone.id, wait)
    }

    def takeoff(DroneWrapper drone) {
        if (drone.state != DroneState.READYINAIR)
            cmdHandler.takeoff(drone.id, null, wait)
        else
            log.warn("Drone ${drone.id} is already in the air. Cannot takeoff.")
    }

    def takeoff(DroneWrapper drone, Number altitude) {
        if (drone.state != DroneState.READYINAIR)
            cmdHandler.takeoff(drone.id, altitude, wait)
        else
            log.warn("Drone ${drone.id} is already in the air. Cannot takeoff.")
    }

    def takeoff(DroneWrapper drone, Distance altitude) {
        if (drone.state != DroneState.READYINAIR)
            cmdHandler.takeoff(drone.id, altitude.convertTo(DistanceUnit.meter), wait)
        else
            log.warn("Drone ${drone.id} is already in the air. Cannot takeoff.")
    }

    def move(DroneWrapper drone, Position pos) {
        move([lat: pos.lat, lon: pos.lon, alt: pos.alt], drone)
    }

    def move(Map m, DroneWrapper drone, Position pos) {
        move(m + [lat: pos.lat, lon: pos.lon, alt: pos.alt], drone)
    }

    def move(DroneWrapper drone, Map m) {
        move(m, drone)
    }

    def move(Map m, DroneWrapper drone) {
        m.remove('bearing')
        def speed = m.remove("speed")
        if (speed != null) {
            if (speed instanceof Speed) {
                speed = ((Speed) speed).getValue(DistanceUnit.meter, TimeUnit.second)
            } else if (!speed instanceof Number)
                throw new InstructionParsingException(InstructionParsingException.State.INVALID_PARAM, [param: "speed", value: speed])
        }
        Number yaw = m.remove("yaw")
        Number lat, lon, alt

        if (m.isEmpty()) {
            return
        } else if (m.containsKey("lat") && m.containsKey("lon")) {
            try {
                (lat, lon, alt) = CommandValidator.validateMoveCoords(m)
            } catch (IllegalArgumentException e) {
                throw new InstructionParsingException(InstructionParsingException.State.INVALID_MOVE, e.message)
            }
            cmdHandler.goTo(drone.id, lat, lon, alt, speed, yaw, wait)
        } else if (m.containsKey("lat") || m.containsKey("lon") || m.containsKey("alt")) {
            throw new InstructionParsingException(InstructionParsingException.State.INVALID_MOVE, "Missing parameters")
        } else {
            Distance x, y, z
            (x, y, z) = calcDistanceVector(m)
            if (x.amount == 0 && y.amount == 0 && z.amount == 0) {
                return
            }
            cmdHandler.move(drone.id, x.convertTo(DistanceUnit.meter),
                    y.convertTo(DistanceUnit.meter), z.convertTo(DistanceUnit.meter), speed, yaw, wait)
        }
    }

    def turn(DroneWrapper drone, Number deg) {
        cmdHandler.turn(drone.id, deg, wait)
    }

    def turn(DroneWrapper drone, Direction dir) {
        switch (dir) {
            case Direction.left:
                turn(drone, -90)
                break
            case Direction.right:
                turn(drone, 90)
                break
            case Direction.north:
                turn(drone, -drone.heading)
                break
            case Direction.east:
                turn(drone, 90 - drone.heading)
                break
            case Direction.west:
                turn(drone, -90 - drone.heading)
                break
            case Direction.south:
                turn(drone, 180 - drone.heading)
                break
            default:
                throw new InstructionParsingException(InstructionParsingException.State.INVALID_DIR, dir)
        }
    }

    def land(DroneWrapper drone) {
        cmdHandler.land(drone.id, wait)
    }

    def home(DroneWrapper drone) {
        cmdHandler.returnToLaunch(drone.id, null, null, wait)
    }

    def home(DroneWrapper drone, Map m) {
        if (m.containsKey('speed')) {
            if (m.speed instanceof Speed)
                m.speed = ((Speed) m.speed).getValue(DistanceUnit.meter, TimeUnit.second)
            else if (!m.speed instanceof Number)
                throw new InstructionParsingException(InstructionParsingException.State.INVALID_PARAM, [param: "speed", value: m.speed])
        }
        if (m.containsKey('alt')) {
            if (m.alt instanceof Distance)
                m.alt = ((Distance) m.alt).convertTo(DistanceUnit.meter)
            else if (!m.alt instanceof Number)
                throw new InstructionParsingException(InstructionParsingException.State.INVALID_PARAM, [param: "alt", value: m.alt])
        }
        cmdHandler.returnToLaunch(drone.id, m.alt, m.speed, wait)
    }

    def home(Map m, DroneWrapper drone) {
        home(drone, m)
    }

    def home(DroneWrapper drone, Object... args) {
        Map m
        args.each { a ->
            if (a instanceof Speed)
                m.speed = a
            else if (a instanceof Distance)
                m.alt = a
            else
                throw new InstructionParsingException(InstructionParsingException.State.UNRECOGNIZED_PARAM, [param: a, cmd: 'home'])
        }
        home(drone, m)
    }

    def home(DroneWrapper drone, Number alt) {
        cmdHandler.returnToLaunch(drone.id, alt, null, wait)
    }

    def home(DroneWrapper drone, Distance alt) {
        home(drone, alt.convertTo(DistanceUnit.meter))
    }

    def cancel(DroneWrapper drone) {
        cmdHandler.cancel(drone.id, wait)
    }

    private calcDistanceVector(Map directions) {
        Distance x, y, z
        use(DistanceCategory) {
            x = 0.m
            y = 0.m
            z = 0.m
        }
        directions.each { entry ->
            try {
                switch (Direction.valueOf(entry.key.toString())) {
                    case Direction.right:
                        x += entry.value
                        break
                    case Direction.left:
                        x -= entry.value
                        break
                    case Direction.forward:
                        y += entry.value
                        break
                    case Direction.backward:
                        y -= entry.value
                        break
                    case Direction.up:
                        z += entry.value
                        break
                    case Direction.down:
                        z -= entry.value
                        break
                    default:
                        throw new InstructionParsingException(InstructionParsingException.State.INVALID_DIR, entry.key.toString())
                }
            } catch (IllegalArgumentException ignored) {
                throw new InstructionParsingException(InstructionParsingException.State.INVALID_MOVE, "$entry.key")
            }
        }
        return [x, y, z]
    }

    def turn(DroneWrapper drone) {
        [by: { Number deg ->
            turn(drone, deg)
        },
         to: { target ->
             if (target instanceof Number)
                 turn(drone, target - drone.heading)
             else if (target instanceof Direction)
                 turn(drone, target)
             else if (target instanceof DroneWrapper || target instanceof Position || target instanceof Map || target instanceof CommandEnum)
                 turn(drone, PositionCategory.bearing(drone, target) - drone.heading)
         }
        ]
    }

    def move(DroneWrapper drone) {
        [at        : { speed ->
            [to        : { coords ->
                if (coords instanceof Position)
                    coords = [lat: coords.lat, lon: coords.lon, alt: coords.alt]
                move(drone, coords + [speed: speed])
            }, forward : { directions ->
                reconstructMove(drone, directions, 'forward', speed)
            }, backward: { directions ->
                reconstructMove(drone, directions, 'backward', speed)
            }, left    : { directions ->
                reconstructMove(drone, directions, 'left', speed)
            }, right   : { directions ->
                reconstructMove(drone, directions, 'right', speed)
            }, up      : { directions ->
                reconstructMove(drone, directions, 'up', speed)
            }, down    : { directions ->
                reconstructMove(drone, directions, 'down', speed)
            }]
        }, to      : { coords ->
            move(drone, coords)
        }, forward : { directions ->
            reconstructMove(drone, directions, 'forward')
        }, backward: { directions ->
            reconstructMove(drone, directions, 'backward')
        }, left    : { directions ->
            reconstructMove(drone, directions, 'left')
        }, right   : { directions ->
            reconstructMove(drone, directions, 'right')
        }, up      : { directions ->
            reconstructMove(drone, directions, 'up')
        }, down    : { directions ->
            reconstructMove(drone, directions, 'down')
        }
        ]
    }

    private reconstructMove(drone, cmd, mainDir, speed = null) {
        def dir = cmd.remove('by')
        cmd.put(mainDir, dir)
        cmd.put('speed', speed)
        move(drone, cmd)
    }
}