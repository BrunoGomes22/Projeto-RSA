package org.nap.fleetman.server.utils

import org.apache.commons.lang3.RandomStringUtils

class MissionUtils {

    static String generateMissionId() {
        RandomStringUtils.randomAlphanumeric(8)
    }

    static String generateTaskId() {
        RandomStringUtils.randomAlphanumeric(4)
    }

    static File writeConfigToFile(String config, File configDir, String fileName, String extension, File file = null) {
        if (file == null) {
            file = new File("$configDir.name/$fileName.$extension")
            if (configDir.exists()) {
                if (file.exists() && !file.isDirectory())
                    file = new File("$configDir.name/${fileName}_${RandomStringUtils.randomAlphanumeric(3)}.$extension")
            } else
                configDir.mkdirs()
        }
        file.write(config)
        return file
    }
}
