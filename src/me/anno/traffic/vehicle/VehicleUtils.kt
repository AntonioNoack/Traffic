package me.anno.traffic.vehicle

import me.anno.traffic.Lane
import me.anno.traffic.Vehicle

fun Vehicle.setOn(lane: Lane, t: Float) {
    lane.getPosition(t.toDouble(), position)
    lane.getRotation(t, rotation)
    rotationY = rotation.getEulerAngleYXZvY()
    updateDirections()

    route.clear()
    route.add(lane)

    routeIndex = 0
    routeIndexF = t
}