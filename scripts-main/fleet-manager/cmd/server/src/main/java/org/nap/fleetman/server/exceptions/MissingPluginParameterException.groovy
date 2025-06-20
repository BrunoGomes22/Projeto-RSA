package org.nap.fleetman.server.exceptions

class MissingPluginParameterException extends Exception {
    MissingPluginParameterException(List<String> generalParameters, Map typeParameters) {
        super(createMessage(generalParameters, typeParameters))
    }

    private static String createMessage(List<String> generalParameters, Map typeParameters) {
        def message = "Missing parameters on plugin configuration - "
        if (generalParameters != null && !generalParameters.isEmpty())
            message += generalParameters
        if (typeParameters != null && !typeParameters.isEmpty()) {
            if (!message.endsWith(' '))
                message += " and "
            message += "plugin type-specific parameters ${typeParameters.toMapString()}"
        }
        message
    }
}
