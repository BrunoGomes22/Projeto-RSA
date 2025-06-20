package org.nap.fleetman.server.mission.dsl

import org.nap.fleetman.server.model.dsl.Speed

class SpeedCategory {
    static Speed plus(Number num, Speed speed) {
        Number result = num + speed.value
        new Speed(result, speed.distUnit, speed.timeUnit)
    }

    static Speed minus(Number num, Speed speed) {
        Number result = num - speed.value
        new Speed(result, speed.distUnit, speed.timeUnit)
    }

    static Speed multiply(Number num, Speed speed) {
        Number result = num * speed.value
        new Speed(result, speed.distUnit, speed.timeUnit)
    }

    static Speed div(Number num, Speed speed) {
        Number result = num / speed.value
        new Speed(result, speed.distUnit, speed.timeUnit)
    }

    static Speed power(Number num, Speed speed) {
        Number result = num**speed.value
        new Speed(result, speed.distUnit, speed.timeUnit)
    }
}