package me.anno.traffic

import me.anno.utils.types.Booleans.hasFlag
import org.joml.AABBf
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import kotlin.math.max
import kotlin.math.min

class Vehicle {

    val route = ArrayList<Lane>()
    var routeIndex = 0 // we start at the first lane

    var prevSegment: Vehicle? = null
    var prevDistance = 5.0

    // size as a AABB, simplified collider
    // x = right/left, y = up/down, z = forward/backward
    val localBounds = AABBf()

    // todo fill in nearby vehicles to prevent crashes
    val nearby = ArrayList<Vehicle>()

    val position = Vector3d()
    val rotation = Quaternionf()

    val prevPosition = Vector3d()
    val prevRotation = Quaternionf()

    val boundsMin = Vector3d()
    val boundsMax = Vector3d()

    val treeBoundsMin = Vector3d()
    val treeBoundsMax = Vector3d()

    var bounciness = 0.5

    val targetVelocity = Vector3d()

    fun update(dt: Float) {
        targetVelocity.set(0.0)
        computeVelocityForDrivingOnLane()
        stopOnNextSection()
        stopIfAnythingTooClose()
        moveOrCrash()

        updateBounds(dt)
    }

    fun computeVelocityForDrivingOnLane() {
        // todo try to drive along road, all units are in meters
        // todo if stuck (car-angle to lane-direction too large), try to turn around
    }

    fun stopOnNextSection() {
        // todo if !route[routeIndex].mayEnterNextLane(route[routeIndex+1]), stop the car
    }

    fun stopIfAnythingTooClose() {
        // todo respect vehicles in front of us (from route[routeIndex and routeIndex+1)
    }

    fun moveOrCrash() {
        // todo step forward,
        //  implement pseudo-physics based on Verlet integration
        // todo also keep distance constraint to prevSegment
    }

    fun updateBounds(dt: Float) {
        val dt1 = max(2f, dt) // 2s tolerance
        boundsMin.set(Double.POSITIVE_INFINITY)
        boundsMax.set(Double.NEGATIVE_INFINITY)
        for (i in 0 until 8) {
            val delta = Vector3f(
                if (i.hasFlag(1)) localBounds.maxX else localBounds.minX,
                if (i.hasFlag(2)) localBounds.maxY else localBounds.minY,
                if (i.hasFlag(4)) localBounds.maxZ else localBounds.minZ
            )
            delta.rotate(rotation)
            boundsMin.min(delta)
            boundsMax.max(delta)
        }

        val factor = dt1 / dt
        val vx = (position.x - prevPosition.x) * factor
        val vy = (position.y - prevPosition.y) * factor
        val vz = (position.z - prevPosition.z) * factor
        boundsMin.add(position).add(max(vx, 0.0), max(vy, 0.0), max(vz, 0.0))
        boundsMax.add(position).add(min(vx, 0.0), min(vy, 0.0), max(vz, 0.0))

        // good enough to find crash-potential vehicles? probably not...
        val extraScanRadius = 10.0
        boundsMin.add(extraScanRadius, treeBoundsMin)
        boundsMax.sub(extraScanRadius, treeBoundsMax)
    }
}