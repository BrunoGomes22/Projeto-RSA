package org.nap.fleetman.server.parameters

import org.nap.fleetman.server.exceptions.ParameterException
import org.ros2.rcljava.RCLJava
import org.ros2.rcljava.client.Client
import org.ros2.rcljava.node.Node
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Service
import rcl_interfaces.msg.Parameter
import rcl_interfaces.msg.ParameterEvent
import rcl_interfaces.msg.ParameterValue
import rcl_interfaces.msg.SetParametersResult
import rcl_interfaces.srv.*

import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.Future

// Store ROS2 nodes' parameters locally, which can be retrieved or set through the parameter endpoints or the parameter
// client, and will be automatically updated if externally changed
@Service
class ParameterTracker {
    private Node node
    // Store the parameter values for each node/drone
    private Map<String, Map> nodeParameters
    // Store the type of the parameters of each node
    private Map<String, Map<String, Byte>> nodeParametersTypes
    private String namespacePrefix
    private static final Logger log = LoggerFactory.getLogger(ParameterTracker.class)

    ParameterTracker() {
        // A new ROS2 node is required, otherwise subscriptions of the Ground Node would crash upon sending a client request
        node = RCLJava.createNode("ground_param_tracker")
        if (node.namespace == "/")
            namespacePrefix = "/"
        else
            namespacePrefix = node.namespace + "/"
        nodeParameters = new ConcurrentHashMap<>()
        nodeParametersTypes = new ConcurrentHashMap<>()
    }

    // To avoid duplication, add the namespace prefix to each node name if it is in the current namespace.
    // A node in the current namespace could be referred without a /
    private String normalizeNodeName(String nodeName) {
        if (!nodeName.startsWith("/"))
            return namespacePrefix + nodeName
        return nodeName
    }

    // Get all parameters of this drone
    Map getParameters(String nodeName) {
        nodeName = normalizeNodeName(nodeName)
        if (nodeParameters.containsKey(nodeName))
            return nodeParameters[nodeName]
        // If this node's parameters aren't yet known, fetch them
        else
            retrieveParameters(nodeName)
    }

    // Get a specific parameter of this drone
    def getParameter(String nodeName, String parameter) {
        nodeName = normalizeNodeName(nodeName)
        def param
        if (nodeParameters.containsKey(nodeName))
            param = nodeParameters[nodeName][parameter]
        else
            param = retrieveParameters(nodeName)[parameter]
        if (param == null)
            throw new ParameterException(ParameterException.Reason.PARAMETER_UNDECLARED, "No parameter '$parameter' for node '$nodeName'", parameter)
        return param
    }

    // Update stored parameter values when receiving a parameter update (called by MessageSubscriber)
    def updateParameter(ParameterEvent event) {
        String loggedNodeName = event.node.startsWith(namespacePrefix) ? event.node.split(namespacePrefix)[1] : event.node
        if (nodeParameters.containsKey(event.node)) {
            def parameters = nodeParameters[event.node]
            for (param in event.getDeletedParameters()) {
                parameters.remove(param.name)
                log.info("Node $loggedNodeName - Parameter '$param.name' removed")
            }
            for (param in event.getChangedParameters() + event.getNewParameters()) {
                def value = ParameterConverter.convertToValue(param.value)
                // Only update if the new value is different from the currently stored value
                if (parameters[param.name] != value) {
                    parameters[param.name] = value
                    log.info("Node $loggedNodeName - Parameter '$param.name' updated to '$value'")
                }
            }
        }
    }

    // Set a single parameter of this node
    def setParameter(String nodeName, String parameter, value) {
        setParameters(nodeName, [(parameter): value])
    }

    // Set multiple parameters of this node
    def setParameters(String nodeName, Map parameters) {
        SetParametersResult result = setParametersAndGetResult(nodeName, parameters)
        if (!result.successful) {
            def msg = "Failed to update '$nodeName' parameters - $result.reason"
            throw new ParameterException(ParameterException.Reason.FAILED_UPDATE, msg)
        }
    }

    // Return whether this node contains this parameter
    boolean parameterExists(String nodeName, String param) {
        nodeName = normalizeNodeName(nodeName)
        try {
            getParameter(nodeName, param)
        } catch (ParameterException ignored) {
            return false
        }
        return true
    }

    // Set the parameters atomically (the parameters will only be set if all are successfully updated)
    SetParametersResult setParametersAndGetResult(String nodeName, Map parameters) throws ParameterException {
        nodeName = normalizeNodeName(nodeName)
        // If this node is not yet registered, retrieve parameters in order to validate parameter type
        if (!nodeParameters.containsKey(nodeName))
            retrieveParameters(nodeName)
        def paramTypes = nodeParametersTypes[nodeName]
        // Initialize client
        Client<SetParametersAtomically> setClient = node.createClient(SetParametersAtomically.class, nodeName + "/set_parameters_atomically")
        SetParametersAtomically_Request setRequest = new SetParametersAtomically_Request()
        List<Parameter> params = []
        parameters.each { String param, val ->
            // Validate whether parameter exists for this node
            if (paramTypes[param] == null) {
                def msg = "Failed to update '$nodeName' parameter '$param' - undeclared parameter"
                throw new ParameterException(ParameterException.Reason.PARAMETER_UNDECLARED, msg, param)
            }
            // Validate parameter types
            try {
                params += ParameterConverter.convertToParameter(param, val, paramTypes[param])
            } catch (MissingMethodException ignored) {
                def msg = "Failed to update '$nodeName' parameter '$param' - value '$val' cannot be cast to correct type"
                throw new ParameterException(ParameterException.Reason.FAILED_UPDATE, msg, param)
            }
        }
        // Send request
        setRequest.setParameters(params)
        checkIfServiceIsAvailable(setClient, nodeName)
        setClient.waitForService()
        Future<SetParametersAtomically_Response> response = setClient.asyncSendRequest(setRequest)
        return response.get().result
    }

    // Retrieve this node's parameters and store locally
    private Map retrieveParameters(String nodeName) {
        nodeName = normalizeNodeName(nodeName)
        // Retrieve parameter names
        def parameterNames = retrieveParametersNames(nodeName)
        // Retrieve parameter values
        def parameters = retrieveParametersValues(nodeName, parameterNames)
        Map nodeParams = new ConcurrentHashMap<>()
        Map nodeParamsTypes = new ConcurrentHashMap<>()
        // Store the parameter values and types
        parameters.eachWithIndex { ParameterValue param, int i ->
            nodeParams[parameterNames[i]] = ParameterConverter.convertToValue(param)
            nodeParamsTypes[parameterNames[i]] = param.type
        }
        nodeParameters[nodeName] = nodeParams
        nodeParametersTypes[nodeName] = nodeParamsTypes
        return nodeParams
    }

    // Request parameter names for this node
    private List<String> retrieveParametersNames(String nodeName) {
        nodeName = normalizeNodeName(nodeName)
        Client<ListParameters> listClient = node.createClient(ListParameters.class, nodeName + "/list_parameters")
        ListParameters_Request listRequest = new ListParameters_Request()
        checkIfServiceIsAvailable(listClient, nodeName)
        listClient.waitForService()
        Future<ListParameters_Response> futureParameters = listClient.asyncSendRequest(listRequest)
        return futureParameters.get().getResult().names
    }

    // Request parameter values for this node
    private List<ParameterValue> retrieveParametersValues(String nodeName, List<String> parameterNames) {
        nodeName = normalizeNodeName(nodeName)
        Client<GetParameters> getClient = node.createClient(GetParameters.class, nodeName + "/get_parameters")
        GetParameters_Request getRequest = new GetParameters_Request()
        getRequest.setNames(parameterNames)
        checkIfServiceIsAvailable(getClient, nodeName)
        getClient.waitForService()
        Future<GetParameters_Response> futureParameters = getClient.asyncSendRequest(getRequest)
        return futureParameters.get().getValues()
    }

    // Check if this node's parameter service is available (unavailable service usually indicates that no node with this
    // name is found)
    private checkIfServiceIsAvailable(Client client, String nodeName) {
        nodeName = normalizeNodeName(nodeName)
        if (!client.isServiceAvailable()) {
            throw new ParameterException(ParameterException.Reason.NODE, "No parameter service available for node '$nodeName'")
        }
    }
}