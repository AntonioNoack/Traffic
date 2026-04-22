package me.anno.traffic

import me.anno.maths.Maths.clamp
import org.joml.AABBf
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.*

class Vehicle {

    companion object {
        private val nextId = AtomicInteger(0)
    }

    val id = nextId.incrementAndGet()

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
        if (dt <= 0f) return
        time += dt

        if (isCrashed) {
            val friction = exp(-dt * 1.5)
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
        check(velocity.isFinite) { "Invalid velocity by $accelF * $dt * $forward" }

        // 2. Lateral resistance (Tires gripping)
        val maxLateralG = 1.0
        val lateralFrictionAccel = -vR / dt
        val limitedLateralAccel = clamp(lateralFrictionAccel, -maxLateralG * 9.81, maxLateralG * 9.81)
        velocity.fma(limitedLateralAccel * dt, right)
        check(velocity.isFinite)

        // 3. Steering and Rotation
        val currentSpeed = velocity.length()
        var targetHeading = atan2(forward.x, forward.z)
        if (currentSpeed > 0.1) {
            val targetLenSq = targetV.lengthSquared()
            if (targetLenSq > 1e-6) {
                val invTargetLen = 1.0 / sqrt(targetLenSq)
                val targetDirX = targetV.x * invTargetLen
                val targetDirZ = targetV.z * invTargetLen
                targetHeading = atan2(targetDirX, targetDirZ)

                val localTargetX = targetDirX * right.x + targetDirZ * right.z
                val localTargetZ = targetDirX * forward.x + targetDirZ * forward.z

                val desiredSteeringAngle = atan2(localTargetX, localTargetZ)
                val steeringSpeed = 2.0
                val targetSteering = clamp(desiredSteeringAngle, -0.6, 0.6)
                val steeringErr = targetSteering - steeringAngle
                steeringAngle += clamp(steeringErr, -steeringSpeed * dt.toDouble(), steeringSpeed * dt.toDouble())
            }
        } else {
            steeringAngle *= max(0.0, 1.0 - dt * 5.0)
        }

        val wheelbase = 2.5
        val currentHeading = atan2(forward.x, forward.z)
        val headingError = atan2(
            sin(targetHeading - currentHeading),
            cos(targetHeading - currentHeading)
        )
        val slipCorrection = if (abs(vR) > abs(vF) * 0.5) headingError * 12.0 else 0.0
        // Use actual speed here so a vehicle can recover from a sideways start
        // and still rotate toward the lane direction while moving.
        val targetOmega = if (currentSpeed > 0.05) (currentSpeed / wheelbase) * tan(steeringAngle) + slipCorrection else 0.0

        // Stabilized (semi-implicit) angular velocity update
        val stiffness = 15.0
        val damping = 10.0
        angularVelocity = (angularVelocity + targetOmega * stiffness * dt) / (1.0 + (stiffness + damping) * dt)

        // 4. Reversing prevention
        if (vF < -0.1 && (time - lastCollisionTime) > 0.5) {
            val stopForce = -vF / dt
            velocity.fma(stopForce * dt, forward)
            check(velocity.isFinite)
        }

        moveOrCrash(dt)

        // 5. Resolve collisions after motion, so frame-to-frame overlap is
        // caught immediately instead of only after one vehicle has already
        // passed through the other.
        resolveCollisions(dt)
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
            // Resolve each pair only once per frame; this method may be called
            // from both vehicles' updates.
            if (id > other.id) continue

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

                val resolveAmount = minOverlap * 0.5 + 1e-3
                position.fma(resolveAmount, pushDir)
                other.position.fma(-resolveAmount, pushDir)

                val relVel = Vector3d(velocity).sub(other.velocity)
                val normalVel = relVel.dot(pushDir)
                if (normalVel < 0.0) {
                    val halfCorrection = normalVel * 0.5
                    velocity.fma(-halfCorrection, pushDir)
                    other.velocity.fma(halfCorrection, pushDir)
                }
                val impactSpeed = relVel.length()
                if (impactSpeed > 4.0 || minOverlap > 0.4) {
                    if (!isCrashed || !other.isCrashed) {
                        isCrashed = true
                        other.isCrashed = true
                        val distXZ = relCenter.lengthXZ()
                        val torqueY =
                            if (distXZ > 1e-4) (relCenter.x * relVel.z - relCenter.z * relVel.x) * 0.5 / distXZ else 0.0
                        angularVelocity += clamp(torqueY, -15.0, 15.0)
                        velocity.fma(impactSpeed * 0.15, pushDir)
                        other.velocity.fma(-impactSpeed * 0.15, pushDir)
                    }
                }
                lastCollisionTime = time
                other.lastCollisionTime = other.time
            }
        }
    }

    private fun computeTargetVelocity(dt: Float): Vector3d {
        val targetV = Vector3d()
        val curr = route.getOrNull(routeIndex) ?: return targetV

        val nextT = curr.getClosestT(position, routeIndexF.toDouble())
        val didAdvance = nextT > 1.0 && routeIndex + 1 < route.size
        val updatedRouteIndex = if (didAdvance) routeIndex + 1 else routeIndex
        val updatedRouteIndexF = if (didAdvance) nextT - 1.0 else nextT
        val updatedCurr = route[updatedRouteIndex]
        val next = route.getOrNull(routeIndex + 1)

        // Target look-ahead position for stable guidance
        val lookAheadT = min(1.0, updatedRouteIndexF + 0.15)
        val pTarget = updatedCurr.getPosition(lookAheadT, 0.0, 0.0, Vector3d())

        // Guidance vector: points from current position to a point on lane center ahead
        val guidance = Vector3d(pTarget).sub(position)
        val guidanceLenSq = guidance.lengthSquared()
        if (guidanceLenSq > 1e-8) {
            guidance.mul(1.0 / sqrt(guidanceLenSq))
        } else {
            guidance.set(0.0, 0.0, 1.0).rotate(updatedCurr.getRotation(routeIndexF, Quaternionf()))
        }

        var desiredSpeed = maxVelocity

        // Stopping at lane end / signals - check if we can enter the next lane
        if (next != null && !curr.mayEnterNextLane(next)) {
            val distToNext = (1.0 - routeIndexF.toDouble()) * curr.approxLength
            val brakingDist = sq(velocity.length()) / (2.0 * 0.8 * 9.81)
            if (distToNext < brakingDist + 2.0) {
                desiredSpeed = 0.0
            }
        } else if (next == null && routeIndex == route.size - 1 && routeIndexF > 0.98) {
            desiredSpeed = 0.0
        }

        // Vehicle following
        val forward = rotation.transform(Vector3d(0.0, 0.0, 1.0))
        val currentSpeed = velocity.length()
        for (other in nearby) {
            val toOther = Vector3d(other.position).sub(position)
            val dot = toOther.dot(forward)
            if (dot > 0 && dot < 35.0) {
                val lateralDist = Vector3d(toOther).fma(-dot, forward).length()
                if (lateralDist < 2.5) {
                    val gap = dot - 4.1
                    val otherSpeed = max(0.0, other.velocity.dot(forward))

                    // IDM-like following
                    val safeGap = 2.5 + currentSpeed * 1.0 // 1.0s gap
                    if (gap < safeGap) {
                        val gapFactor = clamp(gap / safeGap)
                        val speedLimit = otherSpeed * gapFactor
                        desiredSpeed = min(desiredSpeed, speedLimit)
                    }

                    if (gap < -0.1 && (currentSpeed - otherSpeed) > 5.0) {
                        isCrashed = true
                    }
                }
            }
        }

        targetV.set(guidance).mul(desiredSpeed)

        routeIndex = updatedRouteIndex
        routeIndexF = updatedRouteIndexF.toFloat()

        return targetV
    }

    private fun moveOrCrash(dt: Float) {
        prevPosition.set(position)
        prevRotation.set(rotation)
        rotation.rotateY((angularVelocity * dt).toFloat())
        rotation.normalize()
        check(rotation.isFinite) { "Invalid rotation by $angularVelocity * $dt" }
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

        val extraScanRadius = 20.0
        treeBoundsMin.set(boundsMin).sub(extraScanRadius)
        treeBoundsMax.set(boundsMax).add(extraScanRadius)
    }

    private fun sq(x: Double) = x * x
}
