package me.anno.traffic.vehicle

import me.anno.maths.Maths.PIf
import me.anno.traffic.Lane
import me.anno.traffic.Vehicle

fun Vehicle.setOn(lane: Lane, t: Float) {
    lane.getPosition(t.toDouble(), 0.0, 0.0, position)
    lane.getRotation(t, rotation).rotateY(PIf) // todo why is this +180° necessary???
    updateDirections()

    route.clear()
    route.add(lane)

    routeIndex = 0
    routeIndexF = t
}