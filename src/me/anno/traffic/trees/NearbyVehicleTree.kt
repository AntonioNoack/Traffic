package me.anno.traffic.trees

import me.anno.graph.octtree.KdTree
import me.anno.graph.octtree.OctTree
import me.anno.traffic.Vehicle
import org.joml.Vector3d

class NearbyVehicleTree : OctTree<Vehicle>(4) {
    override fun createChild(): KdTree<Vector3d, Vehicle> = NearbyVehicleTree()
    override fun getMin(data: Vehicle): Vector3d = data.nearbyBoundsMin
    override fun getMax(data: Vehicle): Vector3d = data.nearbyBoundsMax
    override fun getPoint(data: Vehicle): Vector3d = data.position
}