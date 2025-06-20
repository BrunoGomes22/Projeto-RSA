package org.nap.fleetman.server.parameters

import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.springframework.stereotype.Service

// Entrypoint for mission scripts to interact with the parameter tracker
@Service
class ParameterClient {
    private ParameterTracker paramTracker

    ParameterClient(ParameterTracker paramTracker) {
        this.paramTracker = paramTracker
    }

    // Set any node's parameter
    def set(String nodeName, String parameter, value) {
        paramTracker.setParameter(nodeName, parameter, value)
    }

    // Set any drone node's parameter
    def set(DroneWrapper drone, String parameter, value) {
        set(drone.id, parameter, value)
    }

    // Set multiple parameters
    def set(Map parameters, String nodeName) {
        paramTracker.setParameters(nodeName, parameters)
    }

    def set(Map parameters, DroneWrapper drone) {
        paramTracker.setParameters(drone.id, parameters)
    }

    // Retrieve node's parameter (e.g. params.drone01 to retrieve node 'drone01' parameters)
    // Any unknown method from this class will hit this call
    def propertyMissing(String node) {
        paramTracker.getParameters(node)
    }

    // Retrieve node's parameter (e.g. params['drone01'] to retrieve node 'drone01' parameters)
    def getAt(String node) {
        paramTracker.getParameters(node)
    }

    def getAt(DroneWrapper drone) {
        paramTracker.getParameters(drone.id)
    }
}