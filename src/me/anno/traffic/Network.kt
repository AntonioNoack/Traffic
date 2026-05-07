package me.anno.traffic

import me.anno.Time
import me.anno.ecs.System
import me.anno.ecs.annotations.DebugProperty
import me.anno.ecs.systems.OnUpdate
import me.anno.engine.debug.DebugAABB
import me.anno.engine.debug.DebugShapes
import me.anno.engine.serialization.NotSerializedProperty
import me.anno.graph.octtree.KdTreePairs.queryPairs
import me.anno.input.Input
import me.anno.maths.Maths.sq
import me.anno.traffic.trees.NearbyVehicleTree
import me.anno.traffic.trees.StreetPointTree
import me.anno.traffic.vehicle.update
import me.anno.traffic.vehicle.updateTreeBounds
import me.anno.ui.UIColors
import org.joml.AABBd
import org.joml.Vector3d

class Network : System(), OnUpdate {

    companion object {
        val streetPointDistance = 5.0
    }

    val vehicles = ArrayList<Vehicle>()
    val crossings = ArrayList<Crossing>()
    val lanes = ArrayList<Lane>()
    val streets = ArrayList<Street>()

    var timeMultiplier: Double
        get() = Time.timeSpeed
        set(value) {
            Time.timeSpeed = value
        }

    @DebugProperty
    val drivenKilometers: Float
        get() = drivenKilometersD.toFloat()

    private var drivenKilometersD = 0.0

    @DebugProperty
    val numVehicles get() = vehicles.size

    @DebugProperty
    @NotSerializedProperty
    var numRemoved = 0

    @DebugProperty
    val numStreets get() = streets.size

    val streetPointTree = StreetPointTree()
    val nearbyVehicleTree = NearbyVehicleTree()

    private fun rebuildVehicleTree() {
        nearbyVehicleTree.clear()
        for (vehicle in vehicles) {
            vehicle.updateTreeBounds()
            nearbyVehicleTree.add(vehicle)
        }
    }

    private fun findCloseVehicles() {
        for (vehicle in vehicles) {
            vehicle.nearby.clear()
        }

        nearbyVehicleTree.queryPairs(0) { a, b ->
            check(a !== b)
            a.nearby.add(b)
            b.nearby.add(a)
            false
        }

        validateNearbyVehicles()

        if (Input.isShiftDown) {
            showVehicleTreeBounds()
            showVehicleBounds()
        }
    }

    fun validateNearbyVehicles() {
        val ab = AABBd()
        val bb = AABBd()
        for (a in vehicles) {
            check(nearbyVehicleTree.containsValue(a))
        }
        for (ai in vehicles.indices) {
            val a = vehicles[ai]
            ab.setMin(a.nearbyBoundsMin)
                .setMax(a.nearbyBoundsMax)
                .addMargin(-1e-6) // floating point accuracy margin
            for (bi in ai + 1 until vehicles.size) {
                val b = vehicles[bi]
                bb.setMin(b.nearbyBoundsMin)
                    .setMax(b.nearbyBoundsMax)
                val shouldBeInside = ab.testAABB(bb)
                if (shouldBeInside) {
                    check(b in a.nearby) {
                        "Missing connection, $ab vs $bb"
                    }
                    check(a in b.nearby)
                }
            }
        }
    }

    private fun showVehicleBounds() {
        for (vehicle in vehicles) {
            val aabb = DebugAABB(
                AABBd()
                    .setMin(vehicle.collisionBoundsMin)
                    .setMax(vehicle.collisionBoundsMax),
                -1, 0f
            )
            DebugShapes.showDebugAABB(aabb)
        }
    }

    private fun showVehicleTreeBounds() {
        for (vehicle in vehicles) {
            val aabb = DebugAABB(
                AABBd()
                    .setMin(vehicle.nearbyBoundsMin)
                    .setMax(vehicle.nearbyBoundsMax),
                UIColors.midOrange, 0f
            )
            DebugShapes.showDebugAABB(aabb)
        }
    }

    override fun onUpdate() {
        update(Time.deltaTime.toFloat())
    }

    fun update(dt: Float) {
        rebuildVehicleTree()
        findCloseVehicles()
        vehicles.update(dt)
        deleteCrashedVehicles()
        accumulateStatistics(dt)
    }

    fun accumulateStatistics(dt: Float) {
        var delta = 0.0
        for (vehicle in vehicles) {
            delta += vehicle.velocity.length()
        }
        drivenKilometersD += delta * dt / 1e3
    }

    fun deleteCrashedVehicles() {
        vehicles.removeIf { vehicle ->
            vehicle.isCrashed &&
                    vehicle.timeSinceCollision > 7f &&
                    vehicle.velocity.lengthSquared() < 1e-6 &&
                    vehicle.remove() && onRemove()
        }
    }

    private fun onRemove(): Boolean {
        numRemoved++
        return true
    }

    fun addVehicle(vehicle: Vehicle) {
        vehicles.add(vehicle)
        nearbyVehicleTree.add(vehicle)
    }

    fun removeVehicle(vehicle: Vehicle) {
        vehicles.remove(vehicle)
        nearbyVehicleTree.remove(vehicle)
        vehicle.remove()
    }

    fun addStreet(street: Street) {
        streets.add(street)
        street.from.streets.add(street)
        street.to.streets.add(street)
        for (lane in street.lanes) {
            addLane(lane)
        }
    }

    fun addLane(lane: Lane) {
        lanes.add(lane)
        lane.from.lanes.add(lane)
        lane.to.lanes.add(lane)
    }

    fun removeStreet(street: Street) {
        streets.remove(street)
        street.from.streets.remove(street)
        street.to.streets.remove(street)
        for (lane in street.lanes) {
            removeLane(lane)
        }
    }

    fun removeLane(lane: Lane) {
        lanes.remove(lane)
        lane.from.lanes.remove(lane)
        lane.to.lanes.remove(lane)
    }

    fun addCrossing(crossing: Crossing) {
        crossings.add(crossing)
    }

    fun removeCrossing(crossing: Crossing) {
        crossings.remove(crossing)
    }

    fun addPoint(point: StreetPoint) {
        streetPointTree.add(point)
    }

    fun removePoint(point: StreetPoint) {
        streetPointTree.remove(point)
    }

    fun getOrPutPoint(position: Vector3d, maxDistance: Double = streetPointDistance): StreetPoint {
        var point = getPoint(position, maxDistance)
        if (point != null) return point

        point = StreetPoint(Vector3d(position))
        addPoint(point)
        return point
    }

    fun getPoint(position: Vector3d, maxDistance: Double = streetPointDistance): StreetPoint? {
        var bestPoint: StreetPoint? = null
        var bestDistanceSq = sq(maxDistance)
        streetPointTree.query(
            Vector3d(position).sub(maxDistance),
            Vector3d(position).add(maxDistance)
        ) { point ->
            val distanceSq = point.position.distanceSquared(position)
            if (distanceSq < bestDistanceSq) {
                bestPoint = point
                bestDistanceSq = distanceSq
            }
            false
        }
        return bestPoint
    }
}
