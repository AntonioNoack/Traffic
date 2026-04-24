package me.anno.traffic

import me.anno.maths.Maths.absClamp
import me.anno.maths.Maths.clamp
import me.anno.maths.Maths.mix
import me.anno.maths.Maths.sq
import me.anno.traffic.utils.addUnique
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

    var route = ArrayList<Lane>()
    var routeIndex = 0
    var routeIndexF = 0f

    var linkToEngine: VehicleLink? = null
    val isTrailer get() = linkToEngine != null

    val localBounds = AABBf()
        .setMin(-0.93f, 0.0f, -2.0f)
        .setMax(+0.93f, 1.2f, +1.9f)
        .addMargin(-0.05f) // be more lenient

    val nearby = ArrayList<Vehicle>()

    val position = Vector3d()
    val rotation = Quaternionf()

    val velocity = Vector3f()
    var angularVelocity = 0f
    var steeringAngle = 0f

    val boundsMin = Vector3d()
    val boundsMax = Vector3d()

    val treeBoundsMin = Vector3d()
    val treeBoundsMax = Vector3d()

    var maxVelocity = 50f / 3.6f // 50km/h

    var isCrashed = false

    var timeSinceCollision = -1f

    fun update(dt: Float) {
        if (!(dt > 0f)) return

        val link = linkToEngine
        if (isCrashed) {
            applyDrivingAfterCrash(dt)
        } else if (link != null) {
            applyTrailerFollowing(link, dt)
        } else {
            applyMindfulDriving(dt)
        }

        applyVelocity(dt)

        // Refresh bounds before collision tests so the SAT uses the current frame's pose,
        // not the previous frame's cached AABB.
        updateStrictBounds()

        // Resolve collisions after motion, so frame-to-frame overlap is caught immediately
        // instead of only after one vehicle has already passed through the other.
        resolveCollisions()
    }

    fun attachTrailer(trailer: Vehicle, fromDist: Float, toDist: Float) {
        trailer.linkToEngine = VehicleLink(this, fromDist, toDist)
        trailer.position.set(position)
            .sub(rotation.transform(Vector3f(0f, 0f, fromDist + toDist)))
        trailer.route = route
    }

    private fun applyDrivingAfterCrash(dt: Float) {
        timeSinceCollision += dt
        val friction = exp(-dt * 1.5f)
        velocity.mul(friction)
        angularVelocity *= friction
    }

    private fun applyMindfulDriving(dt: Float) {
        val targetVelocity = computeTargetVelocity()
        val forward = rotation.transform(Vector3f(0.0, 0.0, 1.0))
        val right = rotation.transform(Vector3f(1.0, 0.0, 0.0))

        val vF = velocity.dot(forward)
        val vR = velocity.dot(right)

        applyRollingVelocity(targetVelocity, right, forward, vR, vF, dt)
        applySteering(targetVelocity, right, forward, vR, vF, dt)

        applyReversingPrevention(vF, forward)
    }

    private fun applyTrailerFollowing(link: VehicleLink, dt: Float) {

        if (link.engine.isCrashed) {
            markAsCrashed()
            timeSinceCollision = link.engine.timeSinceCollision
            return
        }

        val desiredPos = calculateTrailingPosition(link)
        val toTarget = desiredPos.sub(position, Vector3f())

        val scale = 1f / dt // catch up this very frame :3
        toTarget.mul(scale, velocity)

        // Steering
        val damping = 6f
        val velocity = velocity
        if (velocity.lengthSquared() > 1e-3f) {

            val forward = rotation.transform(Vector3f(0f, 0f, 1f))

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

    var maxAcceleration = 0.3f * 9.81f
    var maxDeceleration = 1.0f * 9.81f

    private fun applyRollingVelocity(
        targetVelocity: Vector3f,
        right: Vector3f, forward: Vector3f,
        vR: Float, vF: Float, dt: Float,
    ) {
        // todo implement gears and shifting time
        // 1. Longitudinal control
        val targetDir = if (targetVelocity.lengthSquared() > 1e-6f)
            Vector3f(targetVelocity).normalize()
        else forward

        val speedErr = targetVelocity.length() - velocity.dot(targetDir)
        val accel = clamp(speedErr / dt, -maxDeceleration, maxAcceleration)

        velocity.fma(accel * dt, targetDir)

        // 2. Lateral resistance (Tires gripping)
        val lateralFriction = 8f // tune 5–15
        val lateralAccel = -vR * lateralFriction
        velocity.fma(lateralAccel * dt, right)
    }

    private fun applySteering(
        targetVelocity: Vector3f,
        right: Vector3f, forward: Vector3f,
        vR: Float, vF: Float, dt: Float,
    ) {
        // 3. Steering and Rotation
        val currentSpeed = velocity.length()
        var targetHeading = atan2(forward.x, forward.z)

        val targetLenSq = targetVelocity.lengthSquared()
        if (targetLenSq > 1e-6f) {
            val targetDirX = targetVelocity.x
            val targetDirZ = targetVelocity.z
            targetHeading = atan2(targetDirX, targetDirZ)

            val localTargetX = targetDirX * right.x + targetDirZ * right.z
            val localTargetZ = targetDirX * forward.x + targetDirZ * forward.z

            val desiredSteeringAngle = atan2(localTargetX, localTargetZ)
            val steeringSpeed = 2f
            val targetSteering = clamp(desiredSteeringAngle, -0.6f, 0.6f)
            val steeringErr = targetSteering - steeringAngle
            steeringAngle += clamp(steeringErr, -steeringSpeed * dt, steeringSpeed * dt)

        } else {
            // undo steering
            steeringAngle *= exp(-dt * 5f)
        }

        val targetOmega = calculateTargetOmega(forward, targetHeading, vR, vF, currentSpeed)
        updateAngularVelocity(targetOmega, dt)
    }

    private fun calculateTargetOmega(
        forward: Vector3f, targetHeading: Float,
        vR: Float, vF: Float,
        currentSpeed: Float,
    ): Float {
        val wheelbase = 2.5f
        val currentHeading = atan2(forward.x, forward.z)
        val headingError = atan2(
            sin(targetHeading - currentHeading),
            cos(targetHeading - currentHeading)
        )
        val slipCorrection = if (abs(vR) > abs(vF) * 0.5) headingError * 12f else 0f
        // Use actual speed here so a vehicle can recover from a sideways start
        // and still rotate toward the lane direction while moving.
        return if (currentSpeed > 0.05) (currentSpeed / wheelbase) * tan(steeringAngle) + slipCorrection else 0f
    }

    private fun updateAngularVelocity(targetOmega: Float, dt: Float) {
        // Stabilized (semi-implicit) angular velocity update
        val stiffness = 15f
        val damping = 10f
        angularVelocity = (angularVelocity + targetOmega * stiffness * dt) / (1f + (stiffness + damping) * dt)
    }

    private fun applyReversingPrevention(vF: Float, forward: Vector3f) {
        if (vF < 0f && timeSinceCollision > 0.5f) {
            velocity.fma(-vF, forward)
        }
    }

    private val linkRoot: Vehicle
        get() {
            var root = this
            while (true) {
                root = root.linkToEngine?.engine ?: break
            }
            return root
        }

    private fun isLinkedTo(other: Vehicle): Boolean {
        return linkRoot === other.linkRoot
    }

    private fun resolveCollisions() {
        for (other in nearby) {
            // Resolve each pair only once per frame; this method may be called
            // from both vehicles' updates.
            if (id > other.id || isLinkedTo(other)) continue

            // The neighbor may not have updated its cached bounds yet when we are
            // called from a single vehicle update, so refresh it from the current pose.
            other.updateStrictBounds()

            val overlap = isColliding(other)
            if (overlap != null) {
                val (mtvAxis, minOverlap, relCenter) = overlap
                val pushDir = Vector3f(mtvAxis)
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
            }
        }
    }

    private fun markAsCrashed() {
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

    private fun overlapsBounds(other: Vehicle): Boolean {
        val min = boundsMin
        val max = boundsMax
        val otherMin = other.boundsMin
        val otherMax = other.boundsMax
        return max.x >= otherMin.x && max.y >= otherMin.y && max.z >= otherMin.z &&
                min.x <= otherMax.x && min.y <= otherMax.y && min.z <= otherMax.z
    }

    private fun isColliding(other: Vehicle): Collision? {
        if (!overlapsBounds(other)) return null

        val forwardA = rotation.transform(Vector3d(0.0, 0.0, 1.0))
        val rightA = rotation.transform(Vector3d(1.0, 0.0, 0.0))
        val hXA = localBounds.deltaX * 0.5
        val hZA = localBounds.deltaZ * 0.5
        val centerA = Vector3d(position)
            .fma(localBounds.centerX.toDouble(), rightA)
            .fma(localBounds.centerZ.toDouble(), forwardA)

        val forwardB = other.rotation.transform(Vector3d(0.0, 0.0, 1.0))
        val rightB = other.rotation.transform(Vector3d(1.0, 0.0, 0.0))
        val hXB = other.localBounds.deltaX * 0.5
        val hZB = other.localBounds.deltaZ * 0.5
        val centerB = Vector3d(other.position)
            .fma(other.localBounds.centerX.toDouble(), rightB)
            .fma(other.localBounds.centerZ.toDouble(), forwardB)

        val relCenter = centerB.sub(centerA, Vector3f())
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

        return if (colliding && mtvAxis != null) {
            Collision(mtvAxis, minOverlap, relCenter)
        } else null
    }

    private var currCrossing: Crossing? = null
    private fun setCrossing(cs: CrossingSection?) {
        val prev = currCrossing
        if (cs == null && prev == null) return

        prev?.onRoute?.remove(this)
        currCrossing = if (cs != null) {
            val crossing = cs.crossing
            crossing.onRoute.addUnique(this)
            crossing.onRouteSegment = cs.sectionId
            crossing
        } else null
    }

    private fun computeTargetVelocity(): Vector3f {
        val curr = route.getOrNull(routeIndex) ?: return Vector3f()

        val nextT = curr.getClosestT(position, routeIndexF)
        val didAdvance = nextT > 1f && routeIndex + 1 < route.size
        val next = route.getOrNull(routeIndex + 1)
        val canEnterNextLane = curr.mayEnterNextLane(next)

        val cs = curr.crossingSection
        if (didAdvance && canEnterNextLane &&
            cs != null && cs.mayStopOnSection() &&
            curr.mayEnterNextLane(next)
        ) {
            setCrossing(cs)
        } else if (nextT > 0.9f) {
            // far enough on segment -> clear our claim
            setCrossing(null)
        }

        val updatedRouteIndex = if (didAdvance && canEnterNextLane) routeIndex + 1 else routeIndex
        val updatedRouteIndexF = if (didAdvance && canEnterNextLane) nextT - 1f else min(nextT, 1f)
        val updatedCurr = route[updatedRouteIndex]

        // Target look-ahead position for stable guidance
        val dt = clamp((0.5f * velocity.length() + 3f) / curr.approxLength, 0.01f, 0.2f)
        var lookAheadT = updatedRouteIndexF + dt
        var targetSegment = updatedCurr
        if (lookAheadT > 1f && updatedRouteIndex + 1 < route.size) {
            lookAheadT -= 1f
            targetSegment = route[updatedRouteIndex + 1]
        }
        val pTarget = targetSegment.getPosition(lookAheadT.toDouble(), 0.0, 0.0, Vector3d())

        // Guidance vector: points from current position to a point on lane center ahead
        val guidance = pTarget.sub(position, Vector3f())
        val guidanceLenSq = guidance.lengthSquared()
        if (guidanceLenSq > 1e-8) {
            guidance.div(sqrt(guidanceLenSq))
        } else {
            guidance.set(0f, 0f, 1f)
                .rotate(updatedCurr.getRotation(routeIndexF, Quaternionf()))
        }

        // todo slowly lerp between segments...
        var desiredSpeed = min(curr.maxSpeed, maxVelocity)

        desiredSpeed = stopAtSignals(desiredSpeed, curr, next, canEnterNextLane)
        desiredSpeed = followOtherVehicles(desiredSpeed)

        guidance.mul(desiredSpeed)

        routeIndex = updatedRouteIndex
        routeIndexF = updatedRouteIndexF

        return guidance
    }

    private fun stopAtSignals(
        desiredSpeed: Float,
        curr: Lane, next: Lane?, canEnterNextLane: Boolean
    ): Float {
        var desiredSpeed = desiredSpeed
        // Stopping at lane end / signals - check if we can enter the next lane
        if (next != null && !canEnterNextLane) {
            val distToNext = (1f - routeIndexF) * curr.approxLength
            val distToEnd = position.distance(curr.to.position)
            val brakingDist = sq(velocity.length()) / (2f * 0.8f * 9.81f)
            if (distToNext < brakingDist + 4f || distToEnd < brakingDist + 6f) {
                desiredSpeed = 0f
            }
        } else if (next == null && routeIndex == route.size - 1 && routeIndexF > 0.98) {
            desiredSpeed = 0f
        }
        return desiredSpeed
    }

    fun estimateStoppingDistance(speed: Float): Float {
        /*
        val stoppingTime = speed / acceleration // a = v*t
        return 0.5f * acceleration * sq(stoppingTime) // s = a/2 * t²
        */
        val acceleration = maxDeceleration
        return 0.5f * sq(speed) / acceleration
    }

    fun estimateStoppingTime(speed: Float): Float {
        val acceleration = maxDeceleration
        return speed / acceleration // a = v*t
    }

    private fun followOtherVehicles(desiredSpeed: Float): Float {
        var desiredSpeed = desiredSpeed
        val forward = rotation.transform(Vector3f(0f, 0f, 1f))
        val currentSpeed = velocity.length()

        val brakingDistance = 2f * estimateStoppingDistance(currentSpeed) // 2x for soft braking
        val brakingTime = estimateStoppingTime(currentSpeed)

        for (other in nearby) {
            val toOther = other.position.sub(position, Vector3f())
            val dot = toOther.dot(forward)
            if (dot <= 0f) continue

            // todo bug: long trains don't stop early enough in some cases :(, why??
            val otherSpeed = other.velocity.length()
            val relevantDistance = brakingDistance +
                    (localBounds.deltaZ + other.localBounds.deltaZ) * 0.5f + // one car length
                    1.5f // safety distance when standing
            if (dot < relevantDistance) {

                val safetyDistance = mix(0.2f, 1.2f, clamp(currentSpeed / 20f)) +
                        relevantDistance * 0.1f

                // half of each, plus safety
                // "lerp" between deltaX and deltaZ depending on the relative vehicle angle, and position...
                // val deltaAngle = rotation.getEulerAngleYXZvY() - other.rotation.getEulerAngleYXZvY()
                // val relativeAngle = abs(sin(deltaAngle))
                val pseudoCarDiameter = (localBounds.deltaZ + other.localBounds.deltaZ) * 0.5f + safetyDistance
                val lateralDist = toOther.fmaDistance(dot, forward)
                val gap = dot - pseudoCarDiameter

                // todo only apply this, if the car has waited some time,
                //  and we're not just stopped for a traffic light...
                //if (currentSpeed < 1.0 && otherSpeed < 1e-4 && id < other.id)
                //    continue // ignore other car to resolve deadlocks (?)

                // Predict the other vehicle from its route when possible.
                // Drivers can see the intended path, so route geometry is the better signal.
                var effectiveGap = gap
                var effectiveLateral = lateralDist

                var predictionTime = 0f
                while (predictionTime < brakingTime) {
                    predictionTime += 0.25f

                    val predictedToOther = predictRoutePosition(other, predictionTime).sub(position, Vector3f())
                    val predictedDot = predictedToOther.dot(forward)
                    val predictedLateralDist = predictedToOther.fmaDistance(predictedDot, forward)
                    if (predictedLateralDist < effectiveLateral) {
                        effectiveLateral = predictedLateralDist
                        effectiveGap = predictedDot - pseudoCarDiameter
                    }
                }

                desiredSpeed = adjustDesiredSpeed(
                    currentSpeed, effectiveLateral, effectiveGap,
                    otherSpeed, desiredSpeed
                )
            }
        }
        return desiredSpeed
    }

    fun calculateTrailingPosition(link: VehicleLink): Vector3d {
        // Direction engine is facing
        val engineForward = link.engine.rotation.transform(Vector3f(0f, 0f, 1f))
        val trailerForward = rotation.transform(Vector3f(0f, 0f, 1f))

        // Target position for trailer (behind engine)
        return Vector3d(link.engine.position)
            .fma(-link.linkToEngine, engineForward)
            .fma(-link.linkToTrailer, trailerForward)
    }

    private fun adjustDesiredSpeed(
        currentSpeed: Float,
        effectiveLateral: Float,
        effectiveGap: Float,
        otherSpeed: Float,
        desiredSpeed: Float
    ): Float {
        // IDM-like following
        val safeGap = 2.5f + currentSpeed * 1f // 1.0s gap
        return if (effectiveLateral < 2.5f && effectiveGap < safeGap) {
            val gapFactor = clamp(effectiveGap / safeGap)
            val speedLimit = max(0f, otherSpeed * gapFactor)
            min(desiredSpeed, speedLimit)
        } else desiredSpeed
    }

    private fun Vector3f.fmaDistance(dot: Float, forward: Vector3f): Float {
        val x = x - dot * forward.x
        val y = y - dot * forward.y
        val z = z - dot * forward.z
        return sqrt(x * x + y * y + z * z)
    }

    private fun predictRoutePosition(vehicle: Vehicle, timeAhead: Float): Vector3d {
        var remainingDistance = max(0f, vehicle.velocity.length() * timeAhead)
        var routeIndex = vehicle.routeIndex
        var routeT = clamp(vehicle.routeIndexF)

        while (true) {
            val lane = vehicle.route.getOrNull(routeIndex) ?: return vehicle.position
            val laneLength = max(1e-6f, lane.approxLength)
            val laneRemaining = (1f - routeT) * laneLength
            if (remainingDistance <= laneRemaining || routeIndex >= vehicle.route.lastIndex) {
                val targetT = clamp(routeT + remainingDistance / laneLength)
                return lane.getPosition(targetT.toDouble(), 0.0, 0.0, Vector3d())
            }

            remainingDistance -= laneRemaining
            routeIndex++
            routeT = 0f
        }
    }

    private fun applyVelocity(dt: Float) {
        angularVelocity = absClamp(angularVelocity, 100f)
        rotation.rotateY(angularVelocity * dt)
        rotation.normalize()
        check(rotation.isFinite) { "Invalid rotation by $angularVelocity * $dt" }
        position.fma(dt.toDouble(), velocity)
    }

    private fun updateStrictBounds() {
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
    }

    fun updateTreeBounds() {
        treeBoundsMin.set(boundsMin).sub(3.0)
        treeBoundsMax.set(boundsMax).add(3.0)

        val vl = velocity.length()
        if (vl > 1e-3f) {
            // 2x for smooth braking
            val dt1 = 2f * estimateStoppingDistance(vl) / vl
            val vx = velocity.x * dt1
            val vy = velocity.y * dt1
            val vz = velocity.z * dt1
            if (vx > 0) treeBoundsMax.x += vx else treeBoundsMin.x += vx
            if (vy > 0) treeBoundsMax.y += vy else treeBoundsMin.y += vy
            if (vz > 0) treeBoundsMax.z += vz else treeBoundsMin.z += vz
        }
    }

    fun remove(): Boolean {
        markAsCrashed()
        setCrossing(null)
        return true
    }
}
