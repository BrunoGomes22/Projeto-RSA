package org.nap.fleetman.server.plugins

import groovy.io.FileType
import org.apache.groovy.io.StringBuilderWriter
import org.codehaus.groovy.control.MultipleCompilationErrorsException
import org.nap.fleetman.server.concurrent.ThreadExecutor
import org.nap.fleetman.server.exceptions.InvalidPluginParameterException
import org.nap.fleetman.server.exceptions.MissingPluginParameterException
import org.nap.fleetman.server.model.dsl.DroneWrapper
import org.nap.fleetman.server.utils.BindingGenerator
import org.nap.fleetman.server.utils.MissionUtils
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.beans.factory.annotation.Value
import org.springframework.stereotype.Service

import javax.annotation.PostConstruct
import java.util.concurrent.ConcurrentHashMap

@Service
class PluginManager {
    private static final Logger log = LoggerFactory.getLogger(PluginManager.class)
    private BindingGenerator bindingGen
    private ThreadExecutor threadExecutor
    @Value('${fleetman.plugins.dir:plugins}')
    private File pluginsDirectory
    private Map<String, Plugin> plugins

    PluginManager(BindingGenerator bindingGen, ThreadExecutor threadExecutor) {
        this.bindingGen = bindingGen
        this.threadExecutor = threadExecutor
        plugins = new ConcurrentHashMap<>()
    }

    static String getThisPluginId() {
        Thread.currentThread().pluginId
    }

    static boolean isPluginThread() {
        Thread.currentThread().class == ThreadExecutor.MissionThread.class && Thread.currentThread().pluginId != null
    }

    @PostConstruct
    private loadPlugins() {
        if (pluginsDirectory.exists()) {
            pluginsDirectory.eachFileRecurse(FileType.FILES) { file ->
                def pluginContext = parsePlugin(file)
                if (pluginContext != null) {
                    if (plugins.containsKey(pluginContext.cfg.id))
                        log.warn("Reloaded plugin '${pluginContext.cfg.id}' from file '$file.name'")
                    else
                        log.info("Loaded plugin '${pluginContext.cfg.id}' from file '$file.name'")
                    addPluginConfig(pluginContext.cfg, pluginContext.binding).setFile(file)
                }
            }
        } else
            log.warn("Provided plugins directory does not exist.")
    }

    private parsePlugin(script) {
        def compilerConfig = BindingGenerator.generatePluginCompilerConfiguration()
        PluginBinding binding = bindingGen.generatePluginBinding()
        def shell = new GroovyShell(binding, compilerConfig)
        PluginConfigReaderScript cfgScript = shell.parse(script) as PluginConfigReaderScript
        try {
            cfgScript.run()
            if (cfgScript.plugin != null) {
                PluginValidator.validateMissingParameters(cfgScript.plugin)
                PluginValidator.validatePluginInput(cfgScript.plugin.input)
                return [cfg: cfgScript.plugin, binding: binding]
            }
        } catch (InvalidPluginParameterException | MissingPluginParameterException e) {
            logFailedPluginLoad(cfgScript.plugin, e.message)
        } catch (MultipleCompilationErrorsException e) {
            Writer data = new StringBuilderWriter()
            e.errorCollector.getLastError().write(new PrintWriter(data))
            logFailedPluginLoad(cfgScript.plugin, "compilation errors -${data.toString().split(':')[2].split('\n')[0]}")
        } catch (MissingMethodException e) {
            logFailedPluginLoad(cfgScript.plugin, "error on line ${exceptionLineNumber(e, cfgScript)}" +
                    " - no signature of method $e.method for arguments $e.arguments")
        } catch (MissingPropertyException e) {
            logFailedPluginLoad(cfgScript.plugin, "error on line ${exceptionLineNumber(e, cfgScript)}" +
                    " - no such property '$e.property'")
        }
        return null
    }

    private Plugin addPluginConfig(PluginConfig pluginCfg, PluginBinding binding) {
        switch (pluginCfg.type) {
            case PluginType.MONITOR:
                Plugin plugin = new MonitorPlugin(pluginCfg, binding)
                plugins.put(plugin.id, plugin)
                return plugin
            case PluginType.SCHEDULED:
                Plugin plugin = new ScheduledPlugin(pluginCfg, binding)
                plugins.put(plugin.id, plugin)
                return plugin
        }
        return null
    }

    private static logFailedPluginLoad(PluginConfig pluginCfg, String message) {
        log.error("Failed to load plugin${(pluginCfg != null && pluginCfg.id != null) ? " '$pluginCfg.id'" : ""}: $message")
    }

    private static exceptionLineNumber(Exception e, Script script) {
        e.stackTrace.find { msg -> msg.toString().contains(script.class.name) }.lineNumber
    }

    List<PluginDTO> getPluginDTOList(PluginType type = null) {
        def pluginList = plugins.values()
        if (type != null)
            pluginList = pluginList.findAll { plugin -> plugin.type == type }
        pluginList.collect { plugin -> convertToPluginDTO(plugin) }
    }

    PluginDTO getPluginDTO(String pluginId) {
        convertToPluginDTO(plugins.get(pluginId))
    }

    private static PluginDTO convertToPluginDTO(Plugin plugin) {
        new PluginDTO()
                .pluginId(plugin.id)
                .type(plugin.type)
                .input(plugin.input.collect { var, value ->
                    if (value instanceof List)
                        new PluginInputDTO().property(var).type(value[0].getSimpleName())._default(value[1].toString())
                    else
                        new PluginInputDTO().property(var).type(value.getSimpleName())
                })
                .missions(plugin.missionIds as List)
    }

    Map<String, Object> createPlugin(String script) {
        def pluginContext = parsePlugin(script)
        if (pluginContext != null) {
            if (plugins.containsKey(pluginContext.cfg.id)) {
                log.error("Plugin '${pluginContext.cfg.id}' already exists")
                return [pluginId: pluginContext.cfg.id, success: false]
            } else {
                File file = MissionUtils.writeConfigToFile(script, pluginsDirectory, pluginContext.cfg.id, 'groovy')
                addPluginConfig(pluginContext.cfg, pluginContext.binding).setFile(file)
                log.info("Loaded plugin '${pluginContext.cfg.id}'")
                return [pluginId: pluginContext.cfg.id, success: true]
            }
        }
        return [pluginId: null, success: false]
    }

    // Creates a new plugin or updates plugin that already exists
    // Returns integer result code: 0 - updated, 1 - created, 2 - parsing errors, 3 - provided ID and plugin ID mismatch
    int createOrUpdatePlugin(String pluginId, String script) {
        def pluginContext = parsePlugin(script)
        if (pluginContext != null) {
            if (pluginId != pluginContext.cfg.id) {
                log.error("Provided plugin ID '$pluginId' does not match the plugin configuration's ID '${pluginContext.cfg.id}'")
                return 3
            }
            if (plugins.containsKey(pluginId)) {
                Plugin plugin = plugins.get(pluginId)
                MissionUtils.writeConfigToFile(script, pluginsDirectory, pluginId, 'groovy', plugin.file)
                addPluginConfig(pluginContext.cfg, pluginContext.binding).setFile(plugin.file)
                log.info("Reloaded plugin '$pluginId'")
                return 0
            } else {
                File file = MissionUtils.writeConfigToFile(script, pluginsDirectory, pluginId, 'groovy')
                addPluginConfig(pluginContext.cfg, pluginContext.binding).setFile(file)
                log.info("Loaded plugin '$pluginId'")
                return 1
            }
        }
        return 2
    }

    boolean removePlugin(String pluginId) {
        Plugin plugin = plugins.remove(pluginId)
        if (plugin == null)
            return false
        plugin.file.delete()
        log.info("Removed plugin '$pluginId'")
        return true
    }

    boolean enablePlugin(String pluginId, String missionId, List<DroneWrapper> drones, Map input) {
        Plugin plugin = plugins.get(pluginId)
        if (plugin == null)
            return false
        plugin.enableForMission(missionId, input)

        switch (plugin.type) {
            case PluginType.MONITOR:
                drones.each { drone ->
                    plugin.monitorDrone(drone, missionId)
                }
                break
            case PluginType.SCHEDULED:
                plugin.monitor(drones, missionId)
                break
        }
        return true
    }

    def enablePluginForDrone(DroneWrapper drone, String missionId, Set<String> pluginIds) {
        pluginIds.each { pluginId ->
            def plugin = plugins.get(pluginId)
            switch (plugin.type) {
                case PluginType.MONITOR:
                    plugin.monitorDrone(drone, missionId)
                    break
                case PluginType.SCHEDULED:
                    plugin.addDrone(drone, missionId)
                    break
            }
        }
    }

    int disablePlugin(String pluginId, String missionId) {
        Plugin plugin = plugins.get(pluginId)
        if (plugin == null)
            return -1
        else if (!plugin.disableForMission(missionId)) {
            log.warn("Mission \"$missionId\": Attempt to disable plugin \"${pluginId}\", which is not enabled in this mission")
            return 0
        } else {
            threadExecutor.stopMissionPluginThreads(missionId, pluginId)
            return 1
        }
    }

    def disablePluginForDrone(String missionId, String pluginId, String droneId) {
        Plugin plugin = plugins.get(pluginId)
        switch (plugin.type) {
            case PluginType.MONITOR:
                threadExecutor.stopDronePluginThread(missionId, droneId, pluginId)
                break
            case PluginType.SCHEDULED:
                plugin.removeDrone(droneId, missionId)
                break
        }
    }
}
