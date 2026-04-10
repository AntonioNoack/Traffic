package me.anno.traffic

import me.anno.utils.types.Booleans.hasFlag
import org.joml.AABBf
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f

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

    var bounciness = 0.5

    fun update(dt: Float) {
        // todo try to drive along road, all units are in meters
        // todo if route[routeIndex].mayEnterNextLane(route[routeIndex+1])

        // todo respect vehicles in front of us (from route[routeIndex and routeIndex+1)
        // todo if stuck, try to turn around

        // todo step forward,
        //  implement pseudo-physics based on Verlet integration
        // todo also keep distance constraint to prevSegment

        updateBounds()
    }

    fun updateBounds() {
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
        boundsMin.add(position)
        boundsMax.add(position)
    }
}