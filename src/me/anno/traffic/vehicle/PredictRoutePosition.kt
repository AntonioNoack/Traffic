package me.anno.traffic.vehicle

import me.anno.maths.Maths.clamp
import me.anno.traffic.Vehicle
import org.joml.Vector3d
import kotlin.math.max

fun predictRoutePositionPlusTime(vehicle: Vehicle, timeAhead: Float, minVelocity: Float = 0f): Vector3d {
    val velocity = max(vehicle.velocity.length(), minVelocity)
    val remainingDistance = max(0f, velocity * timeAhead)
    return predictRoutePositionPlusDistance(vehicle, remainingDistance)
}

fun predictRoutePositionPlusDistance(vehicle: Vehicle, distance: Float): Vector3d {
    var remainingDistance = distance
    var routeIndex = vehicle.routeIndex
    var routeT = clamp(vehicle.routeIndexF)

    while (true) {
        val lane = vehicle.route.getOrNull(routeIndex) ?: return vehicle.position
        val laneLength = max(1e-6f, lane.approxLength)
        val laneRemaining = (1f - routeT) * laneLength
        if (remainingDistance <= laneRemaining || routeIndex >= vehicle.route.lastIndex) {
            val targetT = clamp(routeT + remainingDistance / laneLength)
            return lane.getPosition(targetT.toDouble(), Vector3d())
        }

        remainingDistance -= laneRemaining
        routeIndex++
        routeT = 0f
    }
}