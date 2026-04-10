package me.anno.traffic

import me.anno.graph.octtree.KdTreePairs.queryPairs

class Network {

    val vehicles = ArrayList<Vehicle>()
    val crossings = ArrayList<Crossing>()
    val lanes = ArrayList<Lane>()
    val roads = ArrayList<Road>()

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
}