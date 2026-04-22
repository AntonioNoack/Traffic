package me.anno.traffic

import me.anno.maths.Maths.clamp
import org.joml.AABBf
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import kotlin.math.*

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
    var angularVelocity = 0.0
    var steeringAngle = 0.0

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
            val friction = 1.0 - dt * 1.5
            velocity.mul(max(0.0, friction))
            angularVelocity *= max(0.0, friction)
            resolveCollisions(dt)
            moveOrCrash(dt)
            updateBounds(dt)
            return
        }

        val targetV = computeTargetVelocity(dt)

        val forward = rotation.transform(Vector3d(0.0, 0.0, 1.0))
        val right = rotation.transform(Vector3d(1.0, 0.0, 0.0))

        val vF = velocity.dot(forward)
        val vR = velocity.dot(right)

        // 1. Longitudinal control
        val targetVF = targetV.dot(forward)
        val speedErr = targetVF - vF
        val maxAcc = 0.3 * 9.81
        val maxDec = 1.0 * 9.81
        val accelF = clamp(speedErr / dt, -maxDec, maxAcc)
        velocity.fma(accelF * dt, forward)

        // 2. Lateral resistance
        val maxLateralG = 1.0
        val lateralFrictionAccel = -vR / dt
        val limitedLateralAccel = clamp(lateralFrictionAccel, -maxLateralG * 9.81, maxLateralG * 9.81)
        velocity.fma(limitedLateralAccel * dt, right)

        // 3. Steering and Rotation
        if (targetV.lengthSquared() > 1e-4) {
            val localTargetV = Vector3d(targetV).rotate(rotation.conjugate(Quaternionf()))
            val desiredSteeringAngle = atan2(localTargetV.x, localTargetV.z)
            val steeringSpeed = 2.0
            val targetSteering = clamp(desiredSteeringAngle, -0.6, 0.6)
            val steeringErr = targetSteering - steeringAngle
            steeringAngle += clamp(steeringErr, -steeringSpeed * dt.toDouble(), steeringSpeed * dt.toDouble())
        } else {
            steeringAngle *= max(0.0, 1.0 - dt * 5.0)
        }

        val wheelbase = 2.5
        val targetOmega = (vF / wheelbase) * tan(steeringAngle)
        val angularStiffness = 10.0
        val angularDamping = 5.0
        val angularAccel = (targetOmega - angularVelocity) * angularStiffness - angularVelocity * angularDamping
        angularVelocity += angularAccel * dt

        // 4. Resolve Rectangle Collisions
        resolveCollisions(dt)

        // 5. Reversing prevention
        if (vF < -0.1 && (time - lastCollisionTime) > 0.5) {
            val stopForce = -vF / dt
            velocity.fma(stopForce * dt, forward)
        }

        moveOrCrash(dt)
        updateBounds(dt)
    }

    private fun resolveCollisions(dt: Float) {
        val forwardA = rotation.transform(Vector3d(0.0, 0.0, 1.0))
        val rightA = rotation.transform(Vector3d(1.0, 0.0, 0.0))
        val hXA = (localBounds.maxX - localBounds.minX) * 0.5
        val hZA = (localBounds.maxZ - localBounds.minZ) * 0.5
        val centerA = Vector3d(position).fma((localBounds.minX + localBounds.maxX) * 0.5, rightA)
            .fma((localBounds.minZ + localBounds.maxZ) * 0.5, forwardA)

        for (other in nearby) {
            val forwardB = other.rotation.transform(Vector3d(0.0, 0.0, 1.0))
            val rightB = other.rotation.transform(Vector3d(1.0, 0.0, 0.0))
            val hXB = (other.localBounds.maxX - other.localBounds.minX) * 0.5
            val hZB = (other.localBounds.maxZ - other.localBounds.minZ) * 0.5
            val centerB = Vector3d(other.position).fma((other.localBounds.minX + other.localBounds.maxX) * 0.5, rightB)
                .fma((other.localBounds.minZ + other.localBounds.maxZ) * 0.5, forwardB)

            val relCenter = Vector3d(centerB).sub(centerA)
            val axes = arrayOf(rightA, forwardA, rightB, forwardB)
            var minOverlap = Double.POSITIVE_INFINITY
            var mtvAxis: Vector3d? = null

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

            if (colliding && mtvAxis != null) {
                val pushDir = Vector3d(mtvAxis)
                if (relCenter.dot(pushDir) > 0) pushDir.mul(-1.0)
                
                val pushF = pushDir.dot(forwardA)
                val pushR = pushDir.dot(rightA)
                val resolveAmount = minOverlap * 0.5
                position.fma(resolveAmount * pushF, forwardA)
                position.fma(resolveAmount * pushR * 0.15, rightA)

                val relVel = Vector3d(velocity).sub(other.velocity)
                val impactSpeed = relVel.length()
                if (impactSpeed > 4.0 || minOverlap > 0.4) {
                    if (!isCrashed || !other.isCrashed) {
                        isCrashed = true
                        other.isCrashed = true
                        val torqueY = (relCenter.x * relVel.z - relCenter.z * relVel.x) * 0.5 / relCenter.lengthXZ()
                        angularVelocity += clamp(torqueY, -15.0, 15.0)
                        velocity.fma(impactSpeed * 0.3, pushDir)
                    }
                }
                lastCollisionTime = time
            }
        }
    }

    private fun computeTargetVelocity(dt: Float): Vector3d {
        val targetV = Vector3d()
        val curr = route.getOrNull(routeIndex) ?: return targetV

        val nextT = curr.getClosestT(position, routeIndexF.toDouble())
        if (nextT > 1.0 && routeIndex + 1 < route.size) {
            routeIndex++
            routeIndexF = (nextT - 1.0).toFloat()
        } else {
            routeIndexF = nextT.toFloat()
        }

        val updatedCurr = route[routeIndex]
        val laneDir = Vector3d()
        val p0 = updatedCurr.getPosition(routeIndexF.toDouble(), 0.0, 0.0, Vector3d())
        val p1 = updatedCurr.getPosition(min(1.0, routeIndexF.toDouble() + 0.1), 0.0, 0.0, Vector3d())

        if (p0.distanceSquared(p1) > 1e-6) {
            laneDir.set(p1).sub(p0).normalize()
        } else {
            laneDir.set(0.0, 0.0, 1.0).rotate(updatedCurr.getRotation(routeIndexF, Quaternionf()))
        }

        var desiredSpeed = maxVelocity
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

        val forward = rotation.transform(Vector3d(0.0, 0.0, 1.0))
        val currentSpeed = velocity.length()
        for (other in nearby) {
            val toOther = Vector3d(other.position).sub(position)
            val dot = toOther.dot(forward)
            if (dot > 0 && dot < 35.0) {
                val lateralDist = Vector3d(toOther).fma(-dot, forward).length()
                if (lateralDist < 2.2) {
                    val gap = dot - 4.0
                    val otherSpeed = max(0.0, other.velocity.dot(forward))
                    val safeGap = 2.0 + currentSpeed * 0.8
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
        val toLane = Vector3d(p0).sub(position)
        toLane.fma(-toLane.dot(laneDir), laneDir)
        targetV.add(toLane.mul(1.0))
        return targetV
    }

    private fun moveOrCrash(dt: Float) {
        prevPosition.set(position)
        prevRotation.set(rotation)
        rotation.rotateY((angularVelocity * dt).toFloat())
        position.fma(dt.toDouble(), velocity)
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
            boundsMin.min(tmpV)
            boundsMax.max(tmpV)
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