package me.anno.traffic

class Network {

    val vehicles = ArrayList<Vehicle>()
    val crossings = ArrayList<Crossing>()
    val lanes = ArrayList<Lane>()

    val vehicleTree = VehicleTree()

    fun rebuildVehicleTree() {
        vehicleTree.clear()
        for (vehicle in vehicles) {
            vehicleTree.add(vehicle)
        }
    }

    fun update(dt: Float) {
        rebuildVehicleTree()
        for (vehicle in vehicles) {
            vehicle.update(dt)
        }
    }
}