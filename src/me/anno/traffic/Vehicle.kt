package me.anno.traffic

import me.anno.traffic.vehicle.VehicleLink
import me.anno.traffic.vehicle.markAsCrashed
import me.anno.traffic.vehicle.setCrossing
import org.joml.AABBf
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import java.util.concurrent.atomic.AtomicInteger

class Vehicle {

    companion object {
        private val nextId = AtomicInteger(0)
    }

    val id = nextId.incrementAndGet()

    val route = ArrayList<Lane>()
    var routeIndex = 0
    var routeIndexF = 0f
    var addedLaneToRoute = false

    var linkToEngine: VehicleLink? = null
    val isTrailer get() = linkToEngine != null

    val localBounds = AABBf()
        .setMin(-0.93f, 0.0f, -2.0f)
        .setMax(+0.93f, 1.2f, +1.9f)
        .addMargin(-0.05f) // be more lenient

    val nearby = ArrayList<Vehicle>()

    val position = Vector3d()
    val rotation = Quaternionf()
    var rotationY = 0f

    val velocity = Vector3f()
    var angularVelocity = 0f
    var steeringAngle = 0f

    val forward = Vector3f(0f, 0f, 1f)
    val right = Vector3f(1f, 0f, 0f)

    val collisionBoundsMin = Vector3d()
    val collisionBoundsMax = Vector3d()

    val nearbyBoundsMin = Vector3d()
    val nearbyBoundsMax = Vector3d()

    var isCrashed = false
    var timeSinceCollision = -1f

    var maxVelocity = 50f / 3.6f // 50km/h
    var maxAcceleration = 0.3f * 9.81f
    var maxDeceleration = 1.0f * 9.81f

    var currCrossing: Crossing? = null

    fun updateDirections() {
        forward.set(0f, 0f, 1f).rotate(rotation)
        right.set(1f, 0f, 0f).rotate(rotation)
    }

    fun remove(): Boolean {
        markAsCrashed()
        setCrossing(null)
        return true
    }
}
