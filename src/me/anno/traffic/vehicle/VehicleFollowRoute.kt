package me.anno.traffic.vehicle

import me.anno.Time
import me.anno.engine.debug.DebugLine
import me.anno.engine.debug.DebugShapes
import me.anno.maths.Maths.PIf
import me.anno.maths.Maths.TAUf
import me.anno.maths.Maths.clamp
import me.anno.maths.Maths.mix
import me.anno.maths.Maths.sq
import me.anno.traffic.CrossingSection
import me.anno.traffic.Lane
import me.anno.traffic.Vehicle
import me.anno.traffic.utils.addUnique
import me.anno.ui.UIColors
import org.joml.Vector3d
import org.joml.Vector3f
import kotlin.math.*

fun Vehicle.applyMindfulDriving(dt: Float) {
    val targetVelocity = computeTargetVelocity()

    val vF = velocity.dot(forward)
    val vR = velocity.dot(right)

    applyRollingVelocity(targetVelocity, vR, dt)
    applySteering(targetVelocity, vR, vF, dt)

    applyReversingPrevention(vF)
}

private fun Vehicle.computeTargetVelocity(): Vector3f {
    val curr = route.getOrNull(routeIndex) ?: return Vector3f()

    val nextT = curr.getClosestT(position, routeIndexF)
    val next = route.getOrNull(routeIndex + 1)
    val canEnterNextLane = curr.mayEnterNextLane(next)
    val didAdvance = nextT > 1f && routeIndex + 1 < route.size && canEnterNextLane

    updateCrossing(curr, next, nextT, didAdvance)

    // Target look-ahead position for stable guidance
    val pTarget = predictRoutePositionPlusTime(this, 1f, minVelocity = 1f)

    // Guidance vector: points from current position to a point on lane center ahead
    val guidance = pTarget.sub(position, Vector3f())
    val guidanceLenSq = guidance.lengthSquared()
    if (guidanceLenSq > 1e-3f) {
        guidance.div(sqrt(guidanceLenSq))
    } else return guidance.set(0f)

    // todo slowly lerp between segments...
    var desiredSpeed = min(curr.maxSpeed, maxVelocity)

    desiredSpeed = stopAtSignals(desiredSpeed, curr, next, canEnterNextLane)
    desiredSpeed = followOtherVehicles(desiredSpeed)

    guidance.mul(desiredSpeed)

    updateRouteIndices(nextT, didAdvance)

    return guidance
}

fun Vehicle.updateRouteIndices(nextT: Float, didAdvance: Boolean) {
    if (didAdvance) routeIndex++
    routeIndexF = if (didAdvance) nextT - 1f else min(nextT, 1f)
}

fun Vehicle.updateCrossing(
    curr: Lane, next: Lane?, nextT: Float,
    didAdvance: Boolean,
) {
    val cs = curr.crossingSection
    if (didAdvance &&
        cs != null && cs.mayStopOnSection() &&
        curr.mayEnterNextLane(next)
    ) {
        setCrossing(cs)
    } else if (nextT > 0.9f) {
        // far enough on segment -> clear our claim
        setCrossing(null)
    }
}

private fun Vehicle.stopAtSignals(
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

fun Vehicle.estimateStoppingDistance(speed: Float): Float {
    /*
    val stoppingTime = speed / acceleration // a = v*t
    return 0.5f * acceleration * sq(stoppingTime) // s = a/2 * t²
    */
    val effectiveDeceleration = max(0.1f, maxDeceleration - computeSlopeAcceleration())
    return 0.5f * sq(speed) / effectiveDeceleration
}

fun Vehicle.estimateStoppingTime(speed: Float): Float {
    val effectiveDeceleration = max(0.1f, maxDeceleration - computeSlopeAcceleration())
    return speed / effectiveDeceleration // a = v*t
}

private fun Vehicle.followOtherVehicles(desiredSpeed: Float): Float {
    var desiredSpeed = desiredSpeed
    val currentSpeed = velocity.length()

    val brakingDistance = 2f * estimateStoppingDistance(currentSpeed) // 2x for soft braking
    val brakingTime = estimateStoppingTime(currentSpeed)

    for (other in nearby) {
        val toOther = other.position.sub(position, Vector3f())
        val dot = toOther.dot(forward) // assumes vehicle driving forward

        // if our velocity moves us away from the other, skip
        if (dot <= 0f) continue

        // todo bug: long trains don't stop early enough in some cases :(, why??
        val otherSpeed = other.velocity.length()
        val relevantDistance = brakingDistance +
                (localBounds.deltaZ + other.localBounds.deltaZ) * 0.5f + // one car length
                2.5f // safety distance when standing

        val farAway = dot >= relevantDistance
        if (farAway) continue

        DebugShapes.showDebugArrow(
            DebugLine(
                Vector3d(position).fma(1f, forward).apply { y += 2.0 },
                Vector3d(other.position).fma(-1f, other.forward).apply { y += 2.0 },
                UIColors.dodgerBlue, Time.deltaTime.toFloat()
            )
        )

        // half of each, plus safety
        // "lerp" between deltaX and deltaZ depending on the relative vehicle angle, and position...
        // val deltaAngle = rotation.getEulerAngleYXZvY() - other.rotation.getEulerAngleYXZvY()
        // val relativeAngle = abs(sin(deltaAngle))
        val carDiameter = pseudoCarDiameter(other)
        val lateralDist = toOther.fmaDistance(dot, forward)
        val gap = dot - carDiameter

        // todo only apply this, if the car has waited some time,
        //  and we're not just stopped for a traffic light...
        //if (currentSpeed < 1.0 && otherSpeed < 1e-4 && id < other.id)
        //    continue // ignore other car to resolve deadlocks (?)

        // Predict the other vehicle from its route when possible.
        // Drivers can see the intended path, so route geometry is the better signal.
        var effectiveGap = gap
        var minLateral = lateralDist

        var predictionTime = 0f
        while (predictionTime < brakingTime) {
            predictionTime += 0.25f

            val predictedToOther = predictRoutePositionPlusTime(other, predictionTime)
                .sub(position, Vector3f())
                .fma(-predictionTime, velocity)

            val predictedDot = predictedToOther.dot(forward)
            val predictedLateral = predictedToOther.fmaDistance(predictedDot, forward)
            if (predictedLateral < minLateral) {
                minLateral = predictedLateral
                effectiveGap = predictedDot - carDiameter
            }
        }

        desiredSpeed = adjustDesiredSpeed(
            currentSpeed, minLateral, effectiveGap,
            otherSpeed, desiredSpeed
        )
    }
    return desiredSpeed
}

fun Vehicle.pseudoCarDiameter(other: Vehicle): Float {
    val howForward = abs(forward.dot(other.forward))
    val otherBounds = other.localBounds
    val safetyDistance = mix(0.2f, 1.5f, clamp(max(velocity.length(), other.velocity.length()) * 0.2f))
    return safetyDistance + 0.5f * mix(localBounds.deltaX + otherBounds.deltaX, localBounds.deltaZ + otherBounds.deltaZ, howForward)
}

private fun adjustDesiredSpeed(
    currentSpeed: Float,
    effectiveLateral: Float,
    effectiveGap: Float,
    otherSpeed: Float,
    desiredSpeed: Float
): Float {
    // IDM-like following
    val safeGap = 2.5f + currentSpeed * 2f // 2.5m + 2s gap
    return if (effectiveLateral < 2.5f && effectiveGap < safeGap) {
        val gapFactor = clamp(effectiveGap / safeGap)
        val speedLimit = max(0f, otherSpeed * gapFactor)
        min(desiredSpeed, speedLimit)
    } else desiredSpeed
}


private fun Vehicle.applyRollingVelocity(targetVelocity: Vector3f, vR: Float, dt: Float) {
    applyLongitudinalControl(targetVelocity, dt)
    applyLateralResistance(vR, dt)
}

private fun Vehicle.applyLongitudinalControl(targetVelocity: Vector3f, dt: Float) {

    // todo implement gears and shifting time
    val targetDir = if (targetVelocity.lengthSquared() > 1e-6f)
        Vector3f(targetVelocity).normalize()
    else forward

    val actualVelocity = velocity.dot(targetDir)
    val requestedVelocity = targetVelocity.length() - actualVelocity

    // base acceleration request
    var acceleration = requestedVelocity / dt + computeSlopeAcceleration()

    // clamp with asymmetric limits (affected by slope)
    val effectiveAcceleration = maxAcceleration * (1f - max(0f, -forward.y) * 0.7f)
    val effectiveDeceleration = maxDeceleration * (1f - max(0f, forward.y) * 0.5f)

    acceleration = clamp(acceleration, -effectiveDeceleration, effectiveAcceleration)

    velocity.fma(acceleration * dt, targetDir)

    // prevent run-away by internal & air friction
    if (velocity.length() > maxVelocity * 1.3f) {
        velocity.mul(exp(-dt))
    }
}

private fun Vehicle.applyLateralResistance(vR: Float, dt: Float) {
    // Lateral resistance (Tires gripping)
    val lateralFriction = 8f // tune 5–15
    val lateralAccel = -vR * lateralFriction
    velocity.fma(lateralAccel * dt, right)
}

private fun Vehicle.computeSlopeAcceleration(): Float {
    // Project forward onto Y axis -> gives sin(theta)
    val slope = forward.y // already normalized direction

    // gravity component along forward axis
    // negative: uphill (slows you), positive: downhill (pushes you)
    return -9.81f * slope
}

private fun Vehicle.applySteering(
    targetVelocity: Vector3f,
    vR: Float, vF: Float, dt: Float,
) {
    // 3. Steering and Rotation
    val currentSpeed = velocity.length()
    var targetHeading = forward.angleY()

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

    val targetOmega = calculateTargetOmega(targetHeading, vR, vF, currentSpeed)
    updateAngularVelocity(targetOmega, dt)
}

private fun Vehicle.calculateTargetOmega(
    targetHeading: Float,
    vR: Float, vF: Float,
    currentSpeed: Float,
): Float {
    val wheelbase = 2.5f
    val currentHeading = forward.angleY()
    val headingError = fastAngleDiff(targetHeading - currentHeading)
    val slipCorrection = if (abs(vR) > abs(vF) * 0.5) headingError * 12f else 0f
    // Use actual speed here so a vehicle can recover from a sideways start
    // and still rotate toward the lane direction while moving.
    return (currentSpeed / wheelbase) * tan(steeringAngle) + slipCorrection
}

private fun fastAngleDiff(angleDiff: Float): Float {
    var angle = angleDiff
    if (angle < -PIf) angle += TAUf
    else if (angle > PIf) angle -= TAUf
    check(abs(angle) <= 3.15f)
    return angle
}

private fun Vehicle.updateAngularVelocity(targetOmega: Float, dt: Float) {
    // Stabilized (semi-implicit) angular velocity update
    val stiffness = 15f
    val damping = 10f
    angularVelocity = (angularVelocity + targetOmega * stiffness * dt) / (1f + (stiffness + damping) * dt)
}

private fun Vehicle.applyReversingPrevention(vF: Float) {
    if (vF < 0f && timeSinceCollision > 0.5f) {
        velocity.fma(-vF, forward)
    }
}

private fun Vector3f.fmaDistance(dot: Float, forward: Vector3f): Float {
    val x = x - dot * forward.x
    val y = y - dot * forward.y
    val z = z - dot * forward.z
    return sqrt(x * x + y * y + z * z)
}

fun Vehicle.setCrossing(cs: CrossingSection?) {
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