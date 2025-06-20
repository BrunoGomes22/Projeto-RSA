package org.nap.fleetman.server.exceptions

class InvalidPluginParameterException extends Exception {
    InvalidPluginParameterException(boolean duringConfiguration, List<String> extra, Map wrong, Map missing) {
        super(createMessage(duringConfiguration, extra, wrong, missing))
    }

    private static String createMessage(boolean duringConfiguration, List<String> extra, Map wrong, Map missing) {
        def message = "Invalid plugin parameter configuration - "

        if (duringConfiguration) {
            if (!extra.isEmpty())
                message += "too many arguments for parameters: $extra"
            if (!wrong.isEmpty()) {
                if (!message.endsWith(' '))
                    message += "; "
                message += "default parameter value does not match the declared class: ${wrong.toMapString()}"
            }
            if (!missing.isEmpty()) {
                if (!message.endsWith(' '))
                    message += "; "
                message += "argument does not declare a class: ${missing.toMapString()}"
            }
        } else {
            if (!extra.isEmpty())
                message += "parameters $extra are not declared by plugin"
            if (!wrong.isEmpty()) {
                if (!message.endsWith(' '))
                    message += "; "
                message += "parameter values do not match the required type: ${wrong.toMapString()}"
            }
            if (!missing.isEmpty()) {
                if (!message.endsWith(' '))
                    message += "; "
                message += "missing input parameters: ${missing.toMapString()}"
            }
        }
        message
    }
}
