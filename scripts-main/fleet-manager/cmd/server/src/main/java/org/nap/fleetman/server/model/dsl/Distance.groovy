package org.nap.fleetman.server.model.dsl

import groovy.transform.TupleConstructor

@TupleConstructor
class Distance implements Comparable {
    Number amount
    DistanceUnit unit

    Speed div(TimeUnit duration) {
        new Speed(amount, unit, duration)
    }

    Distance plus(Distance distance) {
       plus(distance.convertTo(unit))
    }

    Distance plus(Number distance) {
        Number result = amount + distance
        new Distance(result, unit)
    }

    Distance minus(Distance distance) {
        minus(distance.convertTo(unit))
    }

    Distance minus(Number distance) {
        Number result = amount - distance
        new Distance(result, unit)
    }

    Distance multiply(Distance distance) {
        multiply(distance.convertTo(unit))
    }

    Distance multiply(Number distance) {
        Number result = amount * distance
        new Distance(result, unit)
    }

    Distance power(Distance distance) {
        power(distance.convertTo(unit))
    }

    Distance power(Number distance) {
        Number result = amount ** distance
        new Distance(result, unit)
    }

    Distance div(Distance distance) {
        div(distance.convertTo(unit))
    }

    Distance div(Number distance) {
        Number result = amount / distance
        new Distance(result, unit)
    }

    Distance negative() {
        new Distance(-amount, unit)
    }

    Distance getAbs() {
        new Distance(amount.abs(), unit)
    }

    String toString() {
        "$amount$unit"
    }

    Number convertTo(DistanceUnit unit) {
        if (unit == this.unit)
            return amount
        amount * (this.unit.multiplier / unit.multiplier)
    }

    @Override
    int compareTo(Object o) {
        if (o instanceof Number)
            return this.convertTo(DistanceUnit.meter) <=> (Number) o
        if (o instanceof Distance)
            return amount <=> ((Distance) o).convertTo(unit)
    }
}