package me.anno.traffic

import me.anno.graph.octtree.KdTree
import me.anno.graph.octtree.OctTree
import org.joml.Vector3d

class VehicleTree : OctTree<Vehicle>(16) {
    override fun createChild(): KdTree<Vector3d, Vehicle> = VehicleTree()
    override fun getMin(data: Vehicle): Vector3d = data.boundsMin
    override fun getMax(data: Vehicle): Vector3d = data.boundsMax
    override fun getPoint(data: Vehicle): Vector3d = data.position


}