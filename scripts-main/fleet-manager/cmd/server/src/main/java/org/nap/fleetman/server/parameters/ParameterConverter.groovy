package org.nap.fleetman.server.parameters

import rcl_interfaces.msg.Parameter
import rcl_interfaces.msg.ParameterValue

// Convert ROS2 parameters
class ParameterConverter {
    // Convert parameter value and type to ROS2 Parameter object
    static Parameter convertToParameter(String param, val, byte type) {
        Parameter parameter = new Parameter()
        parameter.name = param
        ParameterValue value = new ParameterValue()
        value.setType(type)
        switch (type) {
            case 1:
                value.setBoolValue(val)
                break
            case 2:
                value.setIntegerValue(val)
                break
            case 3:
                value.setDoubleValue(val)
                break
            case 4:
                value.setStringValue(val)
                break
            case 5:
                value.setByteArrayValue(val)
                break
            case 6:
                value.setBoolArrayValue(val)
                break
            case 7:
                value.setIntegerArrayValue(val)
                break
            case 8:
                value.setDoubleArrayValue(val)
                break
            case 9:
                value.setStringArrayValue(val)
                break
        }
        parameter.value = value
        return parameter
    }

    // Convert ROS2 ParameterValue and return the corresponding value
    static convertToValue(ParameterValue param) {
        switch (param.type) {
            case 1:
                return param.boolValue
            case 2:
                return param.integerValue
            case 3:
                return param.doubleValue
            case 4:
                return param.stringValue
            case 5:
                return param.byteArrayValue
            case 6:
                return param.boolArrayValue
            case 7:
                return param.integerArrayValue
            case 8:
                return param.doubleArrayValue
            case 9:
                return param.stringArrayValue
            default:
                return null
        }
    }
}
