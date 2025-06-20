package org.nap.fleetman.server.exceptions

import org.codehaus.groovy.control.MultipleCompilationErrorsException
import org.codehaus.groovy.runtime.StackTraceUtils
import org.codehaus.groovy.syntax.SyntaxException
import org.nap.fleetman.server.mission.dsl.PositionCategory
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.model.mission.MissionEndMsg
import org.slf4j.Logger
import org.slf4j.LoggerFactory

class ExceptionHandler {
    private static final Logger log = LoggerFactory.getLogger(ExceptionHandler.class)
    private boolean printFullStackTrace

    ExceptionHandler(boolean printFullStackTrace) {
        this.printFullStackTrace = printFullStackTrace
    }

    static private int exceptionLineNumber(Exception e, boolean isPlugin) {
        def st
        if (isPlugin)
            st = e.stackTrace.find { msg -> msg.toString().contains("run_closure") }
        else
            st = e.stackTrace.find { msg -> msg.toString().contains("Script1") }

        if (st != null)
            st.lineNumber
        else
            0
    }

    private Object catchUnexpectedError(Exception e) {
        if (printFullStackTrace) {
            log.debug("Printing full stack trace:")
            e.printStackTrace()
        }
        return ["Encountered unexpected exception on line ${e.stackTrace[0]}: ${e.message}\"", MissionEndMsg.UNEXPECTED]
    }

    Map runAndCatchExceptions(Closure c, boolean isPlugin=false) {
        String logMsg
        MissionEndMsg endMsg
        try {
            c.run()
        } catch (NullPointerException e) {
            StackTraceUtils.sanitize(e)
            if (nullWasDroneWrapper(e)) {
                logMsg = "Reference to revoked drone on line number ${exceptionLineNumber(e, isPlugin)}: ${e.message.split("\n")[0]}"
                endMsg = MissionEndMsg.REVOKED
            } else
                (logMsg, endMsg) = catchUnexpectedError(e)
        } catch (MissingMethodException e) {
            StackTraceUtils.sanitize(e)
            if (isPositionMethod(e.method))
                logMsg = "Invalid method invocation on line " +
                        "${exceptionLineNumber(e, isPlugin)}: missing lat and/or lon arguments on '${e.method}' method call"
            else
                logMsg = "Invalid method invocation on line ${exceptionLineNumber(e, isPlugin)}: ${e.message.split("\n")[0]}"
            endMsg = MissionEndMsg.SCRIPT
        // TODO review if the following catch is sketchy
        } catch (MissingPropertyException e) {
            StackTraceUtils.sanitize(e)
            if (e.stackTrace[0].className.contains('Script1')) {
                logMsg = "Reference to missing property on line ${exceptionLineNumber(e, isPlugin)}: '${e.property}'"
                endMsg = MissionEndMsg.SCRIPT
            } else if (e.property == 'pluginClient') {
                logMsg = "Cannot enable or disable plugins in plugin script"
                endMsg = MissionEndMsg.SCRIPT
            } else (logMsg, endMsg) = catchUnexpectedError(e)
        } catch (MultipleCompilationErrorsException e) {
            e.errorCollector.errors.each {
                if (it.getCause().class == SecurityException.class) {
                    logMsg = "Script accessed class forbidden by security rules - ${it.getCause().getMessage()}"
                    endMsg = MissionEndMsg.SECURITY
                } else if (it.getCause().class == SyntaxException) {
                    logMsg = "Compilation errors on line ${it.cause.line} - ${it.cause.originalMessage}"
                    endMsg = MissionEndMsg.SCRIPT
                } else {
                    if (printFullStackTrace) {
                        log.debug("Printing full stack trace:")
                        e.printStackTrace()
                    }
                    logMsg = "Script encountered unexpected exception - ${it.getCause()}"
                    endMsg = MissionEndMsg.UNEXPECTED
                }
            }
        } catch (SecurityException e) {
            logMsg = "Script attempted illegal operation of manually setting drone id on line ${exceptionLineNumber(e, isPlugin)}"
            endMsg = MissionEndMsg.SECURITY
        } catch (IllegalArgumentException e) {
            StackTraceUtils.sanitize(e)
            if (e.stackTrace[0].className == 'Script1') {
                logMsg = "Illegal argument on line ${exceptionLineNumber(e, isPlugin)}: ${e.getMessage()}"
                endMsg = MissionEndMsg.SCRIPT
            } else
                (logMsg, endMsg) = catchUnexpectedError(e)
        } catch (MissionException e) {
            endMsg = e.missionEndMessage
            logMsg = "Error on line number ${exceptionLineNumber(e, isPlugin)} - " + e.message
        } catch (InvalidPluginParameterException e) {
            endMsg = MissionEndMsg.PLUGIN_PARAM
            logMsg = e.message
        } catch (ParameterException e) {
            endMsg = e.reason == ParameterException.Reason.NODE ? MissionEndMsg.PARAM_NODE : MissionEndMsg.PARAM_FAIL
            logMsg = "Error on line number ${exceptionLineNumber(e, isPlugin)} - " + e.message
        } catch (InterruptedException e) {
           throw e
        } catch (Exception e) {
            StackTraceUtils.sanitize(e)
            (logMsg, endMsg) = catchUnexpectedError(e)
        }

        if (endMsg != null)
            return [endMsg: endMsg, logMsg: logMsg]
        else
            return [:]
    }

    private static boolean nullWasDroneWrapper(Exception e) {
        e.message != null && e.message.startsWith("Cannot get property") &&
            DroneWrapper.declaredFields.findAll {!it.synthetic }.collect { it.name }.contains(e.message.split("'")[1])
    }

    private static boolean isPositionMethod(String method) {
        PositionCategory.declaredMethods.find { it.name == method } != null
    }
}
