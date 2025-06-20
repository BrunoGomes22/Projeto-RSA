package org.nap.fleetman.server.model.dsl

import groovy.transform.TupleConstructor

@TupleConstructor
class Speed implements Comparable {
    Number value
    DistanceUnit distUnit = DistanceUnit.meter
    TimeUnit timeUnit = TimeUnit.second

    String toString() {
        "$value $distUnit/$timeUnit"
    }

    def getValue(distUnit, timeUnit) {
        return  value * ((this.distUnit.multiplier / distUnit.multiplier) / (this.timeUnit.multiplier / timeUnit.multiplier))
    }

    Speed plus(Speed speed) {
        plus(speed.convertTo(this))
    }

    Speed plus(Number speed) {
        Number result = value + speed
        new Speed(result, distUnit, timeUnit)
    }

    Speed minus(Speed speed) {
        minus(speed.convertTo(this))
    }

    Speed minus(Number speed) {
        Number result = value - speed
        new Speed(result, distUnit, timeUnit)
    }

    Speed multiply(Speed speed) {
        multiply(speed.convertTo(this))
    }

    Speed multiply(Number speed) {
        Number result = value * speed
        new Speed(result, distUnit, timeUnit)
    }

    Speed power(Speed speed) {
        power(speed.convertTo(this))
    }

    Speed power(Number speed) {
        Number result = value ** speed
        new Speed(result, distUnit, timeUnit)
    }

    Speed div(Speed speed) {
        div(speed.convertTo(this))
    }

    Speed div(Number speed) {
        Number result = value / speed
        new Speed(result, distUnit, timeUnit)
    }

    Speed negative() {
        new Speed(-value, distUnit, timeUnit)
    }

    Speed getAbs() {
        new Speed(value.abs(), distUnit, timeUnit)
    }

    Number convertTo(DistanceUnit dist, TimeUnit time) {
        if (dist == this.distUnit && time == this.timeUnit)
            return value
        value * ((this.distUnit / dist) / (this.timeUnit / time))
    }
    
    Number convertTo(Speed speed) {
        convertTo(speed.distUnit, speed.timeUnit)
    }

    @Override
    int compareTo(Object o) {
        if (o instanceof Number)
            return this.convertTo(DistanceUnit.meter, TimeUnit.second)<=> (Number) o
        if (o instanceof Speed)
            return value <=> ((Speed) o).convertTo(distUnit, timeUnit)
    }
}
