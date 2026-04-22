package me.anno.traffic

import me.anno.maths.Maths.clamp
import org.joml.AABBf
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import kotlin.math.max
import kotlin.math.min

class Vehicle {

    val route = ArrayList<Lane>()
    var routeIndex = 0
    var routeIndexF = 0f

    var prevSegment: Vehicle? = null
    var prevDistance = 5.0

    val localBounds = AABBf()
        .setMin(-0.93f, 0.0f, -2.0f)
        .setMax(+0.93f, 1.2f, +1.9f)
    val nearby = ArrayList<Vehicle>()

    val position = Vector3d()
    val rotation = Quaternionf()

    val prevPosition = Vector3d()
    val prevRotation = Quaternionf()

    val velocity = Vector3d()

    val boundsMin = Vector3d()
    val boundsMax = Vector3d()

    val treeBoundsMin = Vector3d()
    val treeBoundsMax = Vector3d()

    var bounciness = 0.5
    var maxVelocity = 13.0 // ~50km/h

    var isCrashed = false
    var time = 0.0
    var lastCollisionTime = -10.0

    fun update(dt: Float) {
        time += dt
        if (isCrashed) {
            velocity.set(0.0)
            updateBounds(dt)
            return
        }

        val targetV = computeTargetVelocity(dt)

        // Apply acceleration / deceleration limits
        val diffV = Vector3d(targetV).sub(velocity)
        val len = diffV.length()
        if (len > 1e-6) {
            val currentSpeed = velocity.length()
            val targetSpeed = targetV.length()

            val maxAcc = 0.3 * 9.81 * dt
            val maxDec = 1.0 * 9.81 * dt

            // Braking is more aggressive than acceleration
            val limit = if (targetSpeed < currentSpeed - 0.1) maxDec else maxAcc
            if (len > limit) {
                diffV.mul(limit / len)
            }
            velocity.add(diffV)
        }

        // Prevent reversing under normal conditions
        val forward = Vector3d(0.0, 0.0, 1.0).rotate(rotation)
        val speedForward = velocity.dot(forward)
        if (speedForward < -0.1 && (time - lastCollisionTime) > 0.5) {
            velocity.set(0.0)
        }

        // Resolve collisions (soft pushing to prevent overlap)
        resolveCollisions()

        moveOrCrash(dt)
        updateBounds(dt)
    }

    private fun resolveCollisions() {
        for (other in nearby) {
            val toOther = Vector3d(other.position).sub(position)
            val dist = toOther.length()
            val minGap = 4.05 // Slightly more than car length (4.0)
            if (dist < minGap && dist > 1e-4) {
                val push = (minGap - dist) * 0.1 // Soft push to avoid oscillations
                val pushVec = Vector3d(toOther).mul(-push / dist)
                position.add(pushVec)

                // We don't push the 'other' here to avoid double-pushing since nearby is mutual
                val relVel = Vector3d(velocity).sub(other.velocity).length()
                if (relVel > 5.0 && dist < 3.8) {
                    isCrashed = true
                    other.isCrashed = true
                }
                lastCollisionTime = time
            }
        }
    }

    private fun computeTargetVelocity(dt: Float): Vector3d {
        val targetV = Vector3d()
        val curr = route.getOrNull(routeIndex) ?: return targetV

        // 1. Update route progress
        // Look slightly behind to avoid skipping parts of the lane if pushed back
        val nextT = curr.getClosestT(position, max(0.0, routeIndexF - 0.1))
        if (nextT > 1.0 && routeIndex + 1 < route.size) {
            routeIndex++
            routeIndexF = (nextT - 1.0).toFloat()
        } else {
            routeIndexF = min(1.0, nextT).toFloat()
        }

        val updatedCurr = route[routeIndex]

        // 2. Target direction based on lane tangent
        val laneDir = Vector3d()
        val p0 = updatedCurr.getPosition(routeIndexF.toDouble(), 0.0, 0.0, Vector3d())
        val p1 = updatedCurr.getPosition(min(1.0, routeIndexF.toDouble() + 0.1), 0.0, 0.0, Vector3d())
        if (p0.distanceSquared(p1) > 1e-6) {
            laneDir.set(p1).sub(p0).normalize()
        } else {
            laneDir.set(0.0, 0.0, 1.0).rotate(updatedCurr.getRotation(routeIndexF, Quaternionf()))
        }

        var desiredSpeed = maxVelocity

        // 3. Stopping at red lights / end of route
        val next = route.getOrNull(routeIndex + 1)
        if (next != null && !updatedCurr.mayEnterNextLane(next)) {
            val distToNext = (1.0 - routeIndexF) * updatedCurr.approxLength
            val brakingDist = sq(velocity.length()) / (2.0 * 0.7 * 9.81)
            if (distToNext < brakingDist + 2.5) {
                desiredSpeed = 0.0
            }
        } else if (next == null && routeIndex == route.size - 1 && routeIndexF > 0.98) {
            desiredSpeed = 0.0
        }

        // 4. Vehicle following
        val forward = Vector3d(0.0, 0.0, 1.0).rotate(rotation)
        val currentSpeed = velocity.length()

        for (other in nearby) {
            val toOther = Vector3d(other.position).sub(position)
            val dot = toOther.dot(forward)

            // Only consider vehicles in front
            if (dot > 0) {
                val lateralDist = Vector3d(toOther).fma(-dot, forward).length()
                if (lateralDist < 2.2) {
                    val gap = dot - 4.0
                    val otherSpeed = max(0.0, other.velocity.dot(forward))

                    // Smooth following logic
                    val safeGap = 2.0 + currentSpeed * 0.8 // 0.8s rule
                    if (gap < safeGap) {
                        val gapFactor = clamp((gap - 0.5) / (safeGap - 0.5))
                        val speedLimit = otherSpeed * gapFactor
                        desiredSpeed = min(desiredSpeed, speedLimit)
                    }

                    if (gap < 0.0 && (currentSpeed - otherSpeed) > 5.0) {
                        isCrashed = true
                    }
                }
            }
        }

        targetV.set(laneDir).mul(desiredSpeed)

        // 5. Lateral correction to stay on lane center (but don't reverse)
        val toLane = Vector3d(p0).sub(position)
        toLane.fma(-toLane.dot(laneDir), laneDir)
        targetV.add(toLane.mul(1.0)) // Gentle pull to center

        return targetV
    }

    private fun moveOrCrash(dt: Float) {
        prevPosition.set(position)
        position.fma(dt.toDouble(), velocity)
        if (velocity.lengthXZSquared() > 0.01) {
            rotation.rotationY(velocity.angleY().toFloat())
        }
    }

    fun updateBounds(dt: Float) {
        val dt1 = max(2f, dt)
        boundsMin.set(Double.POSITIVE_INFINITY)
        boundsMax.set(Double.NEGATIVE_INFINITY)

        val tmpV = Vector3f()
        for (i in 0 until 8) {
            tmpV.set(
                if ((i and 1) != 0) localBounds.maxX else localBounds.minX,
                if ((i and 2) != 0) localBounds.maxY else localBounds.minY,
                if ((i and 4) != 0) localBounds.maxZ else localBounds.minZ
            ).rotate(rotation)
            boundsMin.min(tmpV.x.toDouble(), tmpV.y.toDouble(), tmpV.z.toDouble())
            boundsMax.max(tmpV.x.toDouble(), tmpV.y.toDouble(), tmpV.z.toDouble())
        }

        boundsMin.add(position)
        boundsMax.add(position)

        val vx = velocity.x * dt1
        val vy = velocity.y * dt1
        val vz = velocity.z * dt1

        if (vx > 0) boundsMax.x += vx else boundsMin.x += vx
        if (vy > 0) boundsMax.y += vy else boundsMin.y += vy
        if (vz > 0) boundsMax.z += vz else boundsMin.z += vz

        val extraScanRadius = 5.0
        treeBoundsMin.set(boundsMin).sub(extraScanRadius, extraScanRadius, extraScanRadius)
        treeBoundsMax.set(boundsMax).add(extraScanRadius, extraScanRadius, extraScanRadius)
    }

    private fun sq(x: Double) = x * x
}