package org.nap.fleetman.server.mission.dsl

import groovyx.gpars.dataflow.Dataflows
import org.nap.fleetman.server.mission.TaskLauncher
import org.springframework.stereotype.Component

@Component
class UtilCategory {
    static TaskLauncher taskLauncher

    UtilCategory(TaskLauncher taskLauncher) {
        this.taskLauncher = taskLauncher
    }

    static boolean getEven(Number num) {
        return num % 2 == 0
    }

    static boolean getOdd(Number num) {
        return num % 2 != 0
    }

    static void clear(Dataflows df, String var) {
        df.remove(var)
    }

    static boolean getRunning(String var) {
        taskLauncher.running(var)
    }

    static boolean getFinished(String var) {
        taskLauncher.finished(var)
    }
}
