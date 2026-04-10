package me.anno.traffic

import me.anno.graph.octtree.KdTreePairs.queryPairs
import me.anno.maths.Maths.sq
import me.anno.traffic.utils.PointTree
import me.anno.traffic.utils.VehicleTree
import org.joml.Vector3d

class Network {

    val vehicles = ArrayList<Vehicle>()
    val crossings = ArrayList<Crossing>()
    val lanes = ArrayList<Lane>()
    val roads = ArrayList<Road>()
    val points = HashSet<LanePoint>()

    private val pointTree = PointTree()
    private val vehicleTree = VehicleTree()
    private fun rebuildVehicleTree() {
        vehicleTree.clear()
        for (vehicle in vehicles) {
            vehicleTree.add(vehicle)
        }
    }

    private fun findCloseVehicles() {
        for (vehicle in vehicles) {
            vehicle.nearby.clear()
        }
        vehicleTree.queryPairs(0) { a, b ->
            a.nearby.add(b)
            b.nearby.add(a)
            false
        }
    }

    fun update(dt: Float) {
        rebuildVehicleTree()
        findCloseVehicles()
        for (vehicle in vehicles) {
            vehicle.update(dt)
        }
    }

    fun addVehicle(vehicle: Vehicle) {
        vehicles.add(vehicle)
        vehicleTree.add(vehicle)
    }

    fun removeVehicle(vehicle: Vehicle) {
        vehicles.remove(vehicle)
        vehicleTree.remove(vehicle)
    }

    fun addRoad(road: Road) {
        roads.add(road)
        for (lane in road.lanes) {
            addLane(lane)
        }
    }

    fun addLane(lane: Lane) {
        lanes.add(lane)
    }

    fun removeRoad(road: Road) {
        roads.remove(road)
        for (lane in road.lanes) {
            removeLane(lane)
        }
    }

    fun removeLane(lane: Lane) {
        lanes.remove(lane)
    }

    fun addCrossing(crossing: Crossing) {
        crossings.add(crossing)
    }

    fun removeCrossing(crossing: Crossing) {
        crossings.remove(crossing)
    }

    fun addPoint(point: LanePoint) {
        if (points.add(point)) pointTree.add(point)
    }

    fun removePoint(point: LanePoint) {
        if (points.remove(point)) pointTree.remove(point)
    }

    // todo ensure rotation is close, too
    fun getPoint(position: Vector3d, maxDistance: Double): LanePoint? {
        var bestPoint: LanePoint? = null
        var bestDistanceSq = sq(maxDistance)
        pointTree.query(
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