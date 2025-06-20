package org.nap.fleetman.server.model.dsl

enum TimeUnit {
    millisecond('ms', 0.001),
    second('s', 1),
    minute('min', 60),
    hour('h', 3600)

    String abbreviation
    double multiplier

    TimeUnit(String abbr, double mult) {
        this.abbreviation = abbr
        this.multiplier = mult
    }
    String toString() {
        abbreviation
    }
}
