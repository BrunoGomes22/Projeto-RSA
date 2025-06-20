package org.nap.fleetman.server.utils

import org.nap.fleetman.server.model.drone.StatusMessage
import org.slf4j.*

import java.time.Instant
import java.time.LocalDateTime
import java.time.ZoneId
import java.time.format.DateTimeFormatter

class DroneLog {
    private static final Logger log = LoggerFactory.getLogger(DroneLog.class)
    private static final Marker droneMarker = MarkerFactory.getMarker("drones")

    private static void setTimestamp(long timestamp) {
        LocalDateTime droneTimestamp = Instant.ofEpochMilli(timestamp).atZone(ZoneId.systemDefault()).toLocalDateTime()
        MDC.put("timestamp", droneTimestamp.format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss,SSS")))
    }

    static synchronized void info(String droneId, String message, long timestamp) {
        setTimestamp(timestamp)
        log.info(droneMarker, "$droneId - $message")
        MDC.clear()
    }

    static void info(String droneId, String message) {
        info(droneId, message, Instant.now().toEpochMilli())
    }

    static void info(StatusMessage message) {
        info(message.droneId, message.toString(), message.timestamp)
    }

    static synchronized void warn(String droneId, String message, long timestamp) {
        setTimestamp(timestamp)
        log.warn(droneMarker, "$droneId - $message")
        MDC.clear()
    }

    static void warn(String droneId, String message) {
        warn(droneId, message, Instant.now().toEpochMilli())
    }

    static void warn(StatusMessage message) {
        warn(message.droneId, message.toString(), message.timestamp)
    }

    static synchronized void error(String droneId, String message, long timestamp) {
        setTimestamp(timestamp)
        log.error(droneMarker, "$droneId - $message")
        MDC.clear()
    }

    static void error(String droneId, String message) {
        error(droneId, message, Instant.now().toEpochMilli())
    }

    static void error(StatusMessage message) {
        warn(message.droneId, message.toString(), message.timestamp)
    }
}
