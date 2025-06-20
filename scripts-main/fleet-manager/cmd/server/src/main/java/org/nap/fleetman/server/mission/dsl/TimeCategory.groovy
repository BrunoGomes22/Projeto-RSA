package org.nap.fleetman.server.mission.dsl


import org.nap.fleetman.server.model.dsl.Time
import org.nap.fleetman.server.model.dsl.TimeUnit

class TimeCategory {
    static Time getMillisecond(Number num) {
        new Time(num, TimeUnit.millisecond)
    }

    static Time getSecond(Number num) {
        new Time(num, TimeUnit.second)
    }

    static Time getMinute(Number num) {
        new Time(num, TimeUnit.minute)
    }

    static Time getHour(Number num) {
        new Time(num, TimeUnit.hour)
    }

    static Time getMs(Number num) {
        return getMillisecond(num)
    }

    static Time getS(Number num) {
        return getSecond(num)
    }

    static Time getMin(Number num) {
        return getMinute(num)
    }

    static Time getH(Number num) {
        return getHour(num)
    }

    static Time getMilliseconds(Number num) {
        return getMillisecond(num)
    }

    static Time getSeconds(Number num) {
        return getSecond(num)
    }

    static Time getMinutes(Number num) {
        return getMinute(num)
    }

    static Time getHours(Number num) {
        return getHour(num)
    }

    static Time plus(Number num, Time time) {
        Number result = num + time.amount
        new Time(result, time.unit)
    }

    static Time minus(Number num, Time time) {
        Number result = num - time.amount
        new Time(result, time.unit)
    }

    static Time multiply(Number num, Time time) {
        Number result = num * time.amount
        new Time(result, time.unit)
    }

    static Time div(Number num, Time time) {
        Number result = num / time.amount
        new Time(result, time.unit)
    }

    static Time power(Number num, Time time) {
        Number result = num ** time.amount
        new Time(result, time.unit)
    }
}