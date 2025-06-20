package org.nap.fleetman.server.utils

import groovy.transform.ThreadInterrupt
import groovyx.gpars.dataflow.Dataflows
import org.codehaus.groovy.control.CompilerConfiguration
import org.codehaus.groovy.control.customizers.ASTTransformationCustomizer
import org.codehaus.groovy.control.customizers.SecureASTCustomizer
import org.nap.fleetman.server.concurrent.SyncChannel
import org.nap.fleetman.server.mission.CommandParser
import org.nap.fleetman.server.mission.DroneAssigner
import org.nap.fleetman.server.remote.RemoteTaskClient
import org.nap.fleetman.server.mission.MessageClient
import org.nap.fleetman.server.mission.MissionClient
import org.nap.fleetman.server.mission.TaskLauncher
import org.nap.fleetman.server.mission.dsl.MissionBinding
import org.nap.fleetman.server.mission.dsl.MissionHandlerBaseScript
import org.nap.fleetman.server.parameters.ParameterClient
import org.nap.fleetman.server.plugins.PluginBinding
import org.nap.fleetman.server.plugins.PluginClient
import org.nap.fleetman.server.plugins.PluginConfigReaderScript
import org.nap.fleetman.server.plugins.PluginType
import org.nap.fleetman.server.sensors.SensorClient
import org.springframework.context.annotation.Lazy
import org.springframework.stereotype.Service

@Service
class BindingGenerator {
    private SyncChannel syncChannel
    private CommandParser cmdParser
    private DroneAssigner droneAssigner
    private TaskLauncher taskLauncher
    private PluginClient pluginClient
    private RemoteTaskClient remoteTaskClient
    private MessageClient msgClient
    private ParameterClient paramClient
    private SensorClient sensorClient
    private MissionClient missionClient

    BindingGenerator(SyncChannel syncChannel, @Lazy CommandParser cmdParser, @Lazy DroneAssigner droneAssigner,
                     @Lazy TaskLauncher taskLauncher, @Lazy PluginClient pluginClient, @Lazy SensorClient sensorClient,
                     @Lazy MessageClient msgClient, @Lazy RemoteTaskClient remoteTaskClient, @Lazy ParameterClient paramClient,
                     @Lazy MissionClient missionClient) {
        this.syncChannel = syncChannel
        this.cmdParser = cmdParser
        this.droneAssigner = droneAssigner
        this.taskLauncher = taskLauncher
        this.pluginClient = pluginClient
        this.remoteTaskClient = remoteTaskClient
        this.msgClient = msgClient
        this.paramClient = paramClient
        this.sensorClient = sensorClient
        this.missionClient = missionClient
    }

    MissionBinding generateMissionBinding() {
        new MissionBinding(syncChannel,
                cmdParser: cmdParser,
                assigner: droneAssigner,
                taskLauncher: taskLauncher,
                pluginClient: pluginClient,
                remoteTaskClient: remoteTaskClient,
                msgClient: msgClient,
                params: paramClient,
                sensor: sensorClient,
                missionClient: missionClient,
                channel: new Dataflows())
    }

    PluginBinding generatePluginBinding() {
        new PluginBinding(syncChannel,
                cmdParser: cmdParser,
                assigner: droneAssigner,
                taskLauncher: taskLauncher,
                remoteTaskClient: remoteTaskClient,
                msgClient: msgClient,
                params: paramClient,
                sensor: sensorClient,
                missionClient: missionClient,
                *: PluginType.values().collectEntries { [(it.name().toLowerCase()): it] }
        )
    }

    private static SecureASTCustomizer generateBaseSecureASTCustomizer() {
        def secure = new SecureASTCustomizer()
        secure.starImportsWhitelist = ['java.lang.Math', 'org.apache.commons.lang3', 'java.util', 'java.util.concurrent', 'java.util.function']
        secure.setReceiversBlackList(Arrays.asList(System.class.getName()))
        return secure
    }

    private static CompilerConfiguration generateBaseCompilerConfiguration(SecureASTCustomizer secure) {
        def config = new CompilerConfiguration()
        config.addCompilationCustomizers(secure)
        config.addCompilationCustomizers(new ASTTransformationCustomizer(ThreadInterrupt.class))
    }

    static CompilerConfiguration generateMissionCompilerConfiguration() {
        def secure = generateBaseSecureASTCustomizer()
        def compilerConfig = generateBaseCompilerConfiguration(secure)
        compilerConfig.setScriptBaseClass(MissionHandlerBaseScript.name)
        return compilerConfig
    }

    static CompilerConfiguration generatePluginCompilerConfiguration() {
        def secure = generateBaseSecureASTCustomizer()
        secure.setMethodDefinitionAllowed(false)
        def compilerConfig = generateBaseCompilerConfiguration(secure)
        compilerConfig.setScriptBaseClass(PluginConfigReaderScript.name)
        return compilerConfig
    }
}
