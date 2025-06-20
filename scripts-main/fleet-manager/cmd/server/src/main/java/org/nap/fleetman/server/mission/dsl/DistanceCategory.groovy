package org.nap.fleetman.server.mission.dsl


import org.nap.fleetman.server.model.dsl.Distance
import org.nap.fleetman.server.model.dsl.DistanceUnit

class DistanceCategory {
    static Distance getCentimeters(Number num) {
        new Distance(num, DistanceUnit.centimeter)
    }

    static Distance getMeters(Number num) {
        new Distance(num, DistanceUnit.meter)
    }

    static Distance getKilometers(Number num) {
        new Distance(num, DistanceUnit.kilometer)
    }

    static Distance getCm(Number num) {
        getCentimeters(num)
    }

    static Distance getM(Number num) {
        getMeters(num)
    }

    static Distance getKm(Number num) {
        getKilometers(num)
    }

    static Distance getCentimeter(Number num) {
        getCentimeters(num)
    }

    static Distance getMeter(Number num) {
        getMeters(num)
    }

    static Distance getKilometer(Number num) {
        getKilometers(num)
    }

    static Distance plus(Number num, Distance dist) {
        Number result = num + dist.amount
        new Distance(result, dist.unit)
    }

    static Distance minus(Number num, Distance dist) {
        Number result = num - dist.amount
        new Distance(result, dist.unit)
    }

    static Distance multiply(Number num, Distance dist) {
        Number result = num * dist.amount
        new Distance(result, dist.unit)
    }

    static Distance div(Number num, Distance dist) {
        Number result = num / dist.amount
        new Distance(result, dist.unit)
    }

    static Distance power(Number num, Distance dist) {
        Number result = num ** dist.amount
        new Distance(result, dist.unit)
    }
}