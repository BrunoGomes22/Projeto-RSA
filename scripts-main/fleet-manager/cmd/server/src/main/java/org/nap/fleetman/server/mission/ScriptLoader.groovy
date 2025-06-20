package org.nap.fleetman.server.mission

import org.nap.fleetman.server.utils.BindingGenerator
import org.springframework.stereotype.Service

import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

@Service
class ScriptLoader {
    private MissionManager missionMan
    private BindingGenerator bindingGen
    private TaskLauncher taskLauncher

    ScriptLoader(MissionManager missionMan, BindingGenerator bindingGen, TaskLauncher taskLauncher) {
        this.missionMan = missionMan
        this.bindingGen = bindingGen
        this.taskLauncher = taskLauncher
    }

    String loadScript(String script) {
        String missionId = missionMan.initializeMission()
        def binding = bindingGen.generateMissionBinding()
        def config = BindingGenerator.generateMissionCompilerConfiguration()
        def shell = new GroovyShell(this.class.classLoader, binding, config)

        // def scheduler = Executors.newScheduledThreadPool(1)

        taskLauncher.run({
            missionMan.startMission(missionId)
            shell.evaluate(script)
            missionMan.concludeSuccessfulMission(missionId)
        }, missionId: missionId)

        return missionId
    }

}
