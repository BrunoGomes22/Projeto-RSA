package org.nap.fleetman.server.plugins

import org.nap.fleetman.server.exceptions.InvalidPluginParameterException
import org.nap.fleetman.server.exceptions.MissingPluginParameterException

class PluginValidator {
    static void validateMissingParameters(PluginConfig cfg) {
        def generalParameters=[], typeParameters=[:]
        if (cfg.id == null || cfg.id.isBlank())
            generalParameters += 'id'
        if (cfg.type == null)
            generalParameters += 'type'
        else {
            switch (cfg.type) {
                case PluginType.MONITOR:
                    if (cfg.callback == null)
                        typeParameters.put(PluginType.MONITOR, 'callback')
                    break
                case PluginType.SCHEDULED:
                    if (cfg.callback == null)
                        typeParameters.put(PluginType.SCHEDULED, 'callback')
                    break
            }
        }
        if (!(generalParameters.isEmpty() && typeParameters.isEmpty()))
            throw new MissingPluginParameterException(generalParameters, typeParameters)
    }

    static void validatePluginInput(Map input) {
        def extra=[], wrong=[:], missing=[:]
        input.each { var, value ->
            if (value instanceof List) {
                if (value.size() > 2)
                    extra += var
                if (value[0].class != Class)
                    missing.put(var, value[0])
                if (!isValidParamValue(value[0], value[1]))
                    wrong.put(var, value[0].getSimpleName())
            } else if (value.class != Class)
                missing.put(var, value)
        }
        if (!(extra.isEmpty() && wrong.isEmpty() && missing.isEmpty()))
            throw new InvalidPluginParameterException(true, extra, wrong, missing)
    }

    static void validateMissionInput(Map pluginInput, Map missionInput) {
        def extra=[], wrong=[:], missing=[:]

        missionInput.each { var, value ->
            if (!pluginInput.containsKey(var))
                extra += var
            else {
                Class varClass = List.isInstance(pluginInput.get(var)) ? pluginInput.get(var)[0] : pluginInput.get(var)
                if (!isValidParamValue(varClass, value))
                    wrong.put(var, varClass.getSimpleName())
            }
        }

        pluginInput.findAll {var, value -> !missionInput.containsKey(var) && !(value instanceof List)}.each { var, value  ->
            missing.put(var, value.getSimpleName())
        }

        if (!(extra.isEmpty() && wrong.isEmpty() && missing.isEmpty()))
            throw new InvalidPluginParameterException(false, extra, wrong, missing)
    }

    private static boolean isValidParamValue(clazz, value) {
        if (clazz.isInstance(value))
            return true
        else if (clazz == boolean && value instanceof Boolean)
            return true
        else if ((clazz == int || clazz == Integer) && (value instanceof Integer || value instanceof BigInteger))
            return true
        else if ((clazz == float || clazz == Float) && (value instanceof Float || value instanceof Number || value instanceof BigDecimal))
            return true
        else if ((clazz == double || clazz == Double) && (value instanceof Double || value instanceof Number || value instanceof BigDecimal))
            return true
        false
    }
}
