package org.nap.fleetman.server.model.dsl

import groovy.transform.TupleConstructor

@TupleConstructor
class Time implements Comparable {
    Number amount
    TimeUnit unit

    String toString() {
        "$amount$unit"
    }

    Time plus(Time time) {
        plus(time.convertTo(unit))
    }

    Time plus(Number time) {
        Number result = amount + time
        new Time(result, unit)
    }

    Time minus(Time time) {
        minus(time.convertTo(unit))
    }

    Time minus(Number time) {
        Number result = amount - time
        new Time(result, unit)
    }

    Time multiply(Time time) {
        multiply(time.convertTo(unit))
    }

    Time multiply(Number time) {
        Number result = amount * time
        new Time(result, unit)
    }

    Time power(Time time) {
        power(time.convertTo(unit))
    }

    Time power(Number time) {
        Number result = amount ** time
        new Time(result, unit)
    }

    Time div(Time time) {
        div(time.convertTo(unit))
    }

    Time div(Number time) {
        Number result = amount / time
        new Time(result, unit)
    }

    Number convertTo(TimeUnit unit) {
        if (unit == this.unit)
            return amount
        amount * (this.unit.multiplier / unit.multiplier)
    }

    @Override
    int compareTo(Object o) {
        if (o instanceof Number)
            return this.convertTo(TimeUnit.second) <=> (Number) o
        if (o instanceof Time)
            return amount <=> ((Time) o).convertTo(unit)
    }
}