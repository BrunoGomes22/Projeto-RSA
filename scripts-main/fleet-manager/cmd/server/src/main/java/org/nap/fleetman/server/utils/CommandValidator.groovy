package org.nap.fleetman.server.utils

import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.databind.node.ObjectNode
import org.nap.fleetman.server.model.dsl.Distance
import org.nap.fleetman.server.model.dsl.DistanceUnit

class CommandValidator {
    
    static boolean validateJsonCommand(ObjectMapper mapper, ObjectNode cmd) {
        try {
            if (cmd.get("mode").asText() == "action") {
                switch (cmd.get("cmd").asText()) {
                    case "arm":
                    case "disarm":
                    case "land":
                    case "geofence":
                        break
                    case "takeoff":
                    case "return":
                        if (cmd.has("alt") && !cmd.get("alt").isNull() && !cmd.get("alt").isNumber())
                            return false
                        break
                    case "goto":
                        if (!cmd.get("lat").isNumber() || !cmd.get("lon").isNumber()
                                || Math.abs(cmd.get("lat").asDouble()) > 90 || Math.abs(cmd.get("lon").asDouble()) > 180
                                || (cmd.has("alt") && !cmd.get("alt").isNull() && !cmd.get("alt").isNumber())
                                || (cmd.has("yaw") && !cmd.get("yaw").isNull() && !cmd.get("yaw").isNumber())
                                || (cmd.has("speed") && !cmd.get("speed").isNull() && (!cmd.get("speed").isNumber() || cmd.get("speed").asDouble() <= 0)))
                            return false
                        break
                    default:
                        return false
                }
            } else if (cmd.get("mode").asText() == "offboard") {
                switch (cmd.get("cmd").asText()) {
                    case "start_offboard":
                    case "stop_offboard":
                        break
                    case "position_ned":
                    case "velocity_ned":
                        if ((cmd.get("north") != null && !cmd.get("north").isNumber()) || (cmd.get("east") != null && !cmd.get("east").isNumber())
                                || (cmd.get("down") != null && !cmd.get("down").isNumber()) || (cmd.get("yaw") != null && !cmd.get("yaw").isNumber()))
                            return false
                        break
                    case "velocity_body":
                        if ((cmd.get("forward") != null && !cmd.get("forward").isNumber()) || (cmd.get("right") != null && !cmd.get("right").isNumber())
                                || (cmd.get("down") != null && !cmd.get("down").isNumber()) || (cmd.get("yaw") != null && !cmd.get("yaw").isNumber()))
                            return false
                        break
                    case "attitude":
                    case "attitude_rate":
                        if ((cmd.get("roll") != null && !cmd.get("roll").isNumber()) || (cmd.get("pitch") != null && !cmd.get("pitch").isNumber())
                                || (cmd.get("yaw") != null && !cmd.get("yaw").isNumber())
                                || (cmd.get("thrust") != null && (!cmd.get("thrust").isNumber() || cmd.get("thrust").asDouble() > 1 || cmd.get("thrust").asDouble() < 0)))
                            return false
                        break
                    case "actuator_control":
                        if (!cmd.get("group0").isArray() || !cmd.get("group1").isArray())
                            return false
                        double[] group0 = mapper.convertValue(cmd.get("group0"), double[].class)
                        double[] group1 = mapper.convertValue(cmd.get("group1"), double[].class)
                        if (group0.length != 8 || group1.length != 8)
                            return false
                        8.times { i ->
                            if (Math.abs(group0[i]) > 1 || Math.abs(group1[i]) > 1)
                                return false
                        }
                        break
                    default:
                        return false
                }
            } else if (cmd.get("mode").asText() == "custom") {
                switch (cmd.get("cmd").asText()) {
                    case "turn":
                        if (cmd.has("deg") && !cmd.get("deg").isNumber())
                            return false
                        break
                    case "move":
                        if (!cmd.has("x") || !cmd.get("x").isNumber() || !cmd.has("y") || !cmd.get("y").isNumber()
                                || !cmd.has("z") || !cmd.get("z").isNumber() || (cmd.has("yaw") && !cmd.get("yaw").isNumber())
                                || (cmd.has("speed") && !cmd.get("speed").isNumber()))
                            return false
                        break
                    case "cancel":
                        break
                    default:
                        return false
                }
            } else {
                return false
            }
        } catch (NullPointerException | IllegalArgumentException ignored) {
            return false
        }
        return true
    }

    static def validateMoveCoords(Map coords) {
        Number lat = coords.remove("lat")
        Number lon = coords.remove("lon")
        def alt = coords.remove("alt")

        if (!coords.isEmpty())
            throw new IllegalArgumentException("Too many arguments: $coords")

        if (lat == null || lon == null)
            throw new IllegalArgumentException("Null value on coordinates ($lat, $lon)")

        if (lat.abs() > 90)
            throw new IllegalArgumentException("Invalid latitude value")

        if (lon.abs() > 180)
            throw new IllegalArgumentException("Invalid longitude value")

        if (alt instanceof Distance)
            alt = alt.convertTo(DistanceUnit.meter)

        return [lat, lon, alt as Number]
    }
}
