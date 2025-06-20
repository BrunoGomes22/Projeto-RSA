package org.nap.fleetman.server.remote

import friends_interfaces.msg.DroneParam
import friends_interfaces.msg.MissionParam
import friends_interfaces.msg.SensorParam
import friends_interfaces.srv.Missions
import friends_interfaces.srv.Missions_Request
import org.codehaus.groovy.runtime.typehandling.GroovyCastException
import org.nap.fleetman.server.exceptions.MissionException
import org.nap.fleetman.server.mission.MissionManager
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.model.mission.MissionEndMsg
import org.nap.fleetman.server.model.telemetry.Position
import org.ros2.rcljava.client.Client
import org.ros2.rcljava.node.Node

// Builds a ROS2 service request for a remote task
class RemoteTaskRequestBuilder {
    // Creates a service client instance
    static Client createTaskClient(String droneId, RemoteTaskType task, Node node) {
        // If the requested task is a FRIENDS task, create a friends_interfaces.Mission client
        if (task == RemoteTaskType.inspection || task == RemoteTaskType.monitoring || task == RemoteTaskType.scouting)
            return node.createClient(Missions.class, "/$task/" + droneId)
        // New remote tasks that may be developed in the future should be added here
        throw new MissionException(MissionEndMsg.REMOTE_TASK, "No service task '$task' is configured")
    }

    // Creates a request instance
    static createTaskRequest(Map params=null, RemoteTaskType task, RemoteTaskAction action) {
        def request
        // If it is a FRIENDS task
        if (task == RemoteTaskType.inspection || task == RemoteTaskType.monitoring || task == RemoteTaskType.scouting) {
            request = new Missions_Request()
            request.action = action as String
            // When the service request is to start it, add the service parameters
            if (action == RemoteTaskAction.start) {
                params.mtype = task as String
                try {
                    request.dp = createDroneParameters(params.drone ?: [:], params._wrapper)
                    request.mp = createMissionParameters(params.mission ?: [:])
                    request.sp = createSensorParameters(params.sensor ?: [:])
                } catch (GroovyCastException e) {
                    throw new MissionException(MissionEndMsg.REMOTE_TASK, e.message)
                } catch (NumberFormatException e) {
                    throw new MissionException(MissionEndMsg.REMOTE_TASK, "Cannot cast to float - $e.message")
                }
            }
        // New remote task types should be added here
        } else
            throw new MissionException(MissionEndMsg.REMOTE_TASK, "No service task '$task' is configured")
        return request
    }

    private static createDroneParameters(Map params, DroneWrapper drone) {
        DroneParam droneParam = new DroneParam()
        droneParam.droneId = drone.id
        droneParam.dtype = params.dtype ?: 'hexarotor'
        droneParam.dmaxSpeed = (params.dmax_speed == null ? 1.5 : params.dmax_speed) as float
        droneParam.dobstDist = (params.dobst_dist == null ? 2 : params.dobst_dist) as float
        droneParam.dminBat = (params.dmin_bat == null ? 15 : params.dmin_bat) as float
        droneParam.dsize = (params.dsize == null ? 1.75 : params.dsize) as float
        if (params.dhome == null)
            droneParam.setDhome([drone.home.lat as float, drone.home.lon as float])
        else
            droneParam.setDhome(convertCoordToFloatArray(params.dhome))
    }

    private static createMissionParameters(Map params) {
        MissionParam missionParam = new MissionParam()
        missionParam.missionId = params.mission_id ?: MissionManager.thisMissionId
        missionParam.mtype = params.mtype ?: 'other'
        missionParam.mpathAlgo = params.mpath_algo ?: 'boustrophedon'
        if (params.maoi == null)
            throw new MissionException(MissionEndMsg.REMOTE_TASK, "Task requires mission parameter 'maoi'")
        def aoi = []
        params.maoi.each { val ->
            if (val instanceof Number)
                aoi.add(val)
            else
                aoi.addAll(convertCoordToFloatArray(val))
        }
        if (aoi.size() % 2 != 0)
            throw new MissionException(MissionEndMsg.REMOTE_TASK, "Mission AOI must include coordinates in pairs")
        missionParam.setMaoi(aoi as float[])
        if (params.mheight == null)
            throw new MissionException(MissionEndMsg.REMOTE_TASK, "Task requires mission parameter 'mheight'")
        missionParam.mheight = params.mheight as float
        if (params.mstart == null)
            throw new MissionException(MissionEndMsg.REMOTE_TASK, "Task requires mission parameter 'mstart'")
        missionParam.setMstart(convertCoordToFloatArray(params.mstart))
        if (params.mstop == null)
            throw new MissionException(MissionEndMsg.REMOTE_TASK, "Task requires mission parameter 'mstop'")
        missionParam.setMstop(convertCoordToFloatArray(params.mstop))
        return missionParam
    }

    private static createSensorParameters(Map params) {
        SensorParam sensorParam = new SensorParam()
        sensorParam.sensorId = params.sensor_id ?: 'none'
        sensorParam.stype = params.stype ?: 'other'
        sensorParam.sdist = (params.sdist == null ? 5 : params.sdist) as float
        sensorParam.sgrabTime = (params.sgrab_time == null ? 5 : params.sgrab_time) as float
        sensorParam.sspeed = (params.sspeed == null ? 2 : params.sspeed) as float
        sensorParam.sother = (params.sother == null ? 0 : params.sother) as float
        sensorParam.speriod = (params.speriod == null ? 2 : params.speriod) as float
        return sensorParam
    }

    private static float[] convertCoordToFloatArray(param) {
        if (param instanceof List || (param.class != null && param.class.isArray()))
            return param as float[]
        else if (param instanceof Map || param instanceof Position)
            return [param.lat, param.lon] as float[]
        throw new MissionException(MissionEndMsg.REMOTE_TASK, "Cannot convert $param to float array")
    }
}
