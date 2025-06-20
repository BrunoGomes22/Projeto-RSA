package org.nap.fleetman.server.exceptions

class ParameterException extends RuntimeException {
    enum Reason {NODE, PARAMETER_UNDECLARED, FAILED_UPDATE}
    Reason reason
    String param

    ParameterException(Reason reason, String msg, String param=null) {
        super(msg)
        this.reason = reason
        this.param = param
    }
}
