package me.anno.traffic.vehicle

import me.anno.maths.Maths.clamp
import me.anno.traffic.Collision
import me.anno.traffic.Vehicle
import me.anno.traffic.vehicle.updateStrictBounds
import org.joml.Vector3d
import org.joml.Vector3f
import kotlin.math.abs
import kotlin.math.exp

fun Vehicle.resolveCollisions() {
    for (other in nearby) {
        // Resolve each pair only once per frame; this method may be called from both vehicles' updates.
        if (id > other.id || isLinkedTo(other)) continue

        val overlap = isColliding(other, 0f)
        if (overlap != null) {
            val (pushDir, minOverlap, relCenter) = overlap
            if (relCenter.dot(pushDir) > 0) pushDir.negate()

            val resolveAmount = minOverlap * 0.5f + 1e-3f
            position.fma(resolveAmount, pushDir)
            other.position.fma(-resolveAmount, pushDir)

            val relVel = Vector3f(velocity).sub(other.velocity)
            val normalVel = relVel.dot(pushDir)
            if (normalVel < 0f) {
                val halfCorrection = normalVel * 0.5f
                velocity.fma(-halfCorrection, pushDir)
                other.velocity.fma(halfCorrection, pushDir)
            }

            val impactSpeed = relVel.length()
            if (impactSpeed > 4.0 || minOverlap > 0.4) {
                if (!isCrashed || !other.isCrashed) {
                    markAsCrashed()
                    other.markAsCrashed()

                    val distXZ = relCenter.lengthXZ()
                    var torqueY = (relCenter.x * relVel.z - relCenter.z * relVel.x) * 0.5f / (distXZ + 1e-4f)
                    torqueY = clamp(torqueY, -15f, 15f)

                    angularVelocity += torqueY
                    other.angularVelocity -= torqueY

                    velocity.fma(impactSpeed * 0.15f, pushDir)
                    other.velocity.fma(-impactSpeed * 0.15f, pushDir)
                }
            }
            timeSinceCollision = 0f
            other.timeSinceCollision = 0f

            updateStrictBounds()
            other.updateStrictBounds()
        }
    }
}

fun Vehicle.markAsCrashed() {
    var self = this
    while (true) {
        if (self.isCrashed) break

        self.isCrashed = true
        self.timeSinceCollision = 0f
        val prevSelf = self
        self = self.linkToEngine?.engine ?: break
        prevSelf.linkToEngine = null // unlink trailers at crash
    }
}

fun Vehicle.overlapsBounds(other: Vehicle, padding: Float): Boolean {
    val min = collisionBoundsMin
    val max = collisionBoundsMax
    val otherMin = other.collisionBoundsMin
    val otherMax = other.collisionBoundsMax
    val di = 2f * padding
    return max.x >= otherMin.x + di && max.y >= otherMin.y + di && max.z >= otherMin.z + di &&
            min.x + di <= otherMax.x && min.y + di <= otherMax.y && min.z + di <= otherMax.z
}

fun Vehicle.boundsDistance(other: Vehicle): Collision {
    val forwardA = forward
    val rightA = right
    val hXA = localBounds.deltaX * 0.5f
    val hZA = localBounds.deltaZ * 0.5f
    val centerA = Vector3d(position)
        .fma(localBounds.centerX, rightA)
        .fma(localBounds.centerZ, forwardA)

    val forwardB = other.forward
    val rightB = other.right
    val hXB = other.localBounds.deltaX * 0.5f
    val hZB = other.localBounds.deltaZ * 0.5f
    val centerB = Vector3d(other.position)
        .fma(other.localBounds.centerX, rightB)
        .fma(other.localBounds.centerZ, forwardB)

    val relCenter = centerB.sub(centerA, Vector3f())
    val axes = arrayOf(rightA, forwardA, rightB, forwardB)

    var minOverlap = Float.POSITIVE_INFINITY
    var minSeparation = Float.POSITIVE_INFINITY
    var mtvAxis: Vector3f? = null
    var separatingAxis: Vector3f? = null

    var colliding = true

    for (axis in axes) {
        val distProj = abs(relCenter.dot(axis))
        val radiusA = abs(rightA.dot(axis)) * hXA + abs(forwardA.dot(axis)) * hZA
        val radiusB = abs(rightB.dot(axis)) * hXB + abs(forwardB.dot(axis)) * hZB

        val overlap = radiusA + radiusB - distProj
        if (overlap > 0.0f) {
            // penetration
            if (overlap < minOverlap) {
                minOverlap = overlap
                mtvAxis = axis
            }
        } else {
            // separation
            colliding = false
            val separation = -overlap
            if (separation < minSeparation) {
                minSeparation = separation
                separatingAxis = axis
            }
        }
    }

    return if (colliding) {
        // positive overlap = penetration depth
        Collision(mtvAxis!!, minOverlap, relCenter)
    } else {
        // negative overlap = gap between objects
        Collision(separatingAxis!!, -minSeparation, relCenter)
    }
}

private fun Vehicle.isColliding(other: Vehicle, padding: Float): Collision? {
    if (!overlapsBounds(other, padding)) return null

    val forwardA = forward
    val rightA = right
    val hXA = localBounds.deltaX * 0.5f + padding
    val hZA = localBounds.deltaZ * 0.5f + padding
    val centerA = Vector3d(position)
        .fma(localBounds.centerX, rightA)
        .fma(localBounds.centerZ, forwardA)

    val forwardB = other.forward
    val rightB = other.right
    val hXB = other.localBounds.deltaX * 0.5f + padding
    val hZB = other.localBounds.deltaZ * 0.5f + padding
    val centerB = Vector3d(other.position)
        .fma(other.localBounds.centerX, rightB)
        .fma(other.localBounds.centerZ, forwardB)

    val relCenter = centerB.sub(centerA, Vector3f())
    val axes = arrayOf(rightA, forwardA, rightB, forwardB)
    var minOverlap = Float.POSITIVE_INFINITY
    var mtvAxis: Vector3f? = null

    var colliding = true
    for (axis in axes) {
        val distProj = abs(relCenter.dot(axis))
        val radiusA = abs(rightA.dot(axis)) * hXA + abs(forwardA.dot(axis)) * hZA
        val radiusB = abs(rightB.dot(axis)) * hXB + abs(forwardB.dot(axis)) * hZB
        val overlap = radiusA + radiusB - distProj
        if (overlap <= 0.0) {
            colliding = false
            break
        }
        if (overlap < minOverlap) {
            minOverlap = overlap
            mtvAxis = axis
        }
    }

    return if (colliding && mtvAxis != null) {
        Collision(mtvAxis, minOverlap, relCenter)
    } else null
}


fun Vehicle.applyDrivingAfterCrash(dt: Float) {
    timeSinceCollision += dt
    val friction = exp(-dt * 1.5f)
    velocity.mul(friction)
    angularVelocity *= friction
}
