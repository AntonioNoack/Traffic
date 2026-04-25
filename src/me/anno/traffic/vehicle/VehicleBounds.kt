package me.anno.traffic.vehicle

import me.anno.traffic.Vehicle
import me.anno.utils.pooling.JomlPools

fun Vehicle.updateStrictBounds() {
    collisionBoundsMin.set(Double.POSITIVE_INFINITY)
    collisionBoundsMax.set(Double.NEGATIVE_INFINITY)
    val tmpV = JomlPools.vec3f.borrow()
    for (i in 0 until 8) {
        tmpV.set(
            if ((i and 1) != 0) localBounds.maxX else localBounds.minX,
            if ((i and 2) != 0) localBounds.maxY else localBounds.minY,
            if ((i and 4) != 0) localBounds.maxZ else localBounds.minZ
        ).rotate(rotation)
        collisionBoundsMin.min(tmpV)
        collisionBoundsMax.max(tmpV)
    }
    collisionBoundsMin.add(position)
    collisionBoundsMax.add(position)
}

fun Vehicle.updateTreeBounds() {
    nearbyBoundsMin.set(collisionBoundsMin).sub(3.0)
    nearbyBoundsMax.set(collisionBoundsMax).add(3.0)

    if (isTrailer) return // done, trailers just follow

    val vl = velocity.length()
    if (vl > 1e-3f) {
        // 2x for smooth braking
        val dt1 = 2f * estimateStoppingDistance(vl) / vl
        val added = predictRoutePositionPlusTime(this, dt1)

        val vx = added.x - position.x
        val vy = added.y - position.y
        val vz = added.z - position.z
        if (vx > 0) nearbyBoundsMax.x += vx else nearbyBoundsMin.x += vx
        if (vy > 0) nearbyBoundsMax.y += vy else nearbyBoundsMin.y += vy
        if (vz > 0) nearbyBoundsMax.z += vz else nearbyBoundsMin.z += vz
    }
}
