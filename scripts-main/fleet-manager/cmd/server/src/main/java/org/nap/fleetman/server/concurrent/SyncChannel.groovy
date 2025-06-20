package org.nap.fleetman.server.concurrent

import groovyx.gpars.dataflow.Dataflows
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.springframework.stereotype.Component

import org.slf4j.Logger
import org.slf4j.LoggerFactory

@Component
class SyncChannel {
    private final Dataflows lock
    private static final Logger log = LoggerFactory.getLogger(SyncChannel.class)

    SyncChannel() {
        lock = new Dataflows()
    }

    void waitCommand(String droneId, String cmd) {
        String cmdId = droneId + cmd

        Thread currentThread = Thread.currentThread();

        log.info("Current thread name: " + currentThread.getName());
        boolean isMissionThread = currentThread.getName().startsWith("mission-thread");

        if (lock.contains(cmdId))
            lock.remove(cmdId)

        if (isMissionThread)
            Thread.currentThread().currentCommandDroneId = droneId
        
        lock.getProperty(cmdId)
        lock.remove(cmdId)
        
        if (isMissionThread)
            Thread.currentThread().currentCommandDroneId = null
    }

    def unlockCommand(String droneId, String cmd) {
        lock.setProperty(droneId + cmd, null)
    }

    def waitTelem(DroneWrapper dw) {
        def timestamp = dw.timestamp
        while (timestamp == dw.timestamp)
            lock.getProperty(dw.id + "telem")
    }

    def unlockTelem(String droneId) {
        lock.setProperty(droneId + "telem", true)
        lock.remove(droneId + "telem")
    }

    def waitRemoteTask(String droneId, String task) {
        lock.getProperty(droneId + task)
    }

    def unlockRemoteTask(String droneId, String task, boolean success) {
        lock.setProperty(droneId + task, success)
        lock.remove(droneId + task)
    }

    // Wait until sensor has new data available
    def waitSensor(String type, String sensorId, String droneId) {
        lock.getProperty(type + sensorId + droneId)
    }

    // Notify that new sensor data is available
    def unlockSensor(String type, String sensorId, String droneId, boolean timeout) {
        // Wake threads waiting for this specific sensor type, id, and drone combination
        lock.setProperty(type + sensorId + droneId, timeout)
        // Wake threads waiting for this sensor type and drone, but accepting any ID
        lock.setProperty(type + null + droneId, timeout)
        // Wake threads waiting for this sensor type
        lock.setProperty(type + null + null, timeout)
        // Clear dataflows variable to be empty for the next message from this sensor
        lock.remove(type + sensorId + droneId)
        lock.remove(type + null + droneId)
        lock.remove(type + null + null)
    }
}
