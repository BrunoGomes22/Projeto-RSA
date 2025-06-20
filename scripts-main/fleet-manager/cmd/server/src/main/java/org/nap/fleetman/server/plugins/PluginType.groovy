package org.nap.fleetman.server.plugins

import com.fasterxml.jackson.annotation.JsonValue

enum PluginType {
    MONITOR("monitor"),
    SCHEDULED("scheduled")

    private String value

    PluginType(String value) {
        this.value = value
    }

    @Override
    @JsonValue
    String toString() {
        String.valueOf(value)
    }
}