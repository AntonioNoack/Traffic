package me.anno.traffic.vehicle

import me.anno.maths.Maths.clamp
import me.anno.traffic.Vehicle
import org.joml.Vector3d
import org.joml.Vector3f
import kotlin.math.atan2
import kotlin.math.exp

fun Vehicle.applyTrailerFollowing(link: VehicleLink, dt: Float) {
    if (link.engine.isCrashed) {
        markAsCrashed()
        timeSinceCollision = link.engine.timeSinceCollision
        return
    }

    applyTrailerPosition(link, dt)
    applyTrailerSteering(dt)
    updateTrailerRouteIndices()
}

private fun Vehicle.applyTrailerPosition(link: VehicleLink, dt: Float) {
    val desiredPos = calculateTrailingPosition(forward, link)
    val toTarget = desiredPos.sub(position, Vector3f())

    val scale = 1f / dt // catch up this very frame :3
    toTarget.mul(scale, velocity)
}

private fun Vehicle.applyTrailerSteering(dt: Float) {
    val damping = 6f
    val velocity = velocity
    if (velocity.lengthSquared() > 1e-3f) {

        val headingError = -atan2(
            forward.x * velocity.z - forward.z * velocity.x, // cross
            forward.x * velocity.x + forward.z * velocity.z // dot
        )

        val stiffness = 10f
        val targetOmega = headingError * stiffness * clamp(velocity.length() * 0.3f)
        angularVelocity += targetOmega * dt
        angularVelocity *= exp(-(damping + targetOmega) * dt)

    } else {
        angularVelocity *= exp(-damping * dt)
    }
}

fun calculateTrailingPosition(trailerForward: Vector3f, link: VehicleLink): Vector3d {
    // Direction engine is facing
    val engineForward = link.engine.forward

    // Target position for trailer (behind engine)
    return Vector3d(link.engine.position)
        .fma(-link.linkToEngine, engineForward)
        .fma(-link.linkToTrailer, trailerForward)
}

private fun Vehicle.updateTrailerRouteIndices() {
    val curr = route.getOrNull(routeIndex) ?: return

    val nextT = curr.getClosestT(position, routeIndexF)
    val didAdvance = nextT > 1f && routeIndex + 1 < route.size
    val next = route.getOrNull(routeIndex + 1)

    updateCrossing(curr, next, nextT, didAdvance)
    updateRouteIndices(nextT, didAdvance)
}

private val Vehicle.rootEngine: Vehicle
    get() {
        var root = this
        while (true) {
            root = root.linkToEngine?.engine ?: break
        }
        return root
    }

fun Vehicle.isLinkedTo(other: Vehicle): Boolean {
    return rootEngine === other.rootEngine
}

fun Vehicle.attachTrailer(trailer: Vehicle, fromDist: Float, toDist: Float) {
    trailer.linkToEngine = VehicleLink(this, fromDist, toDist)
    trailer.position.set(predictRoutePositionPlusDistance(this, fromDist + toDist))

    // todo rotate with curve
    trailer.rotation.set(rotation)
    trailer.rotationY = rotationY

    // todo update these based on the distance/closest point...
    trailer.routeIndex = routeIndex
    trailer.routeIndexF = routeIndexF
    trailer.route.addAll(route)

    trailer.updateDirections()
}

