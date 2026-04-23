package me.anno.traffic.utils

import me.anno.graph.octtree.KdTree
import me.anno.graph.octtree.OctTree
import me.anno.traffic.Vehicle
import org.joml.Vector3d

class VehicleTree : OctTree<Vehicle>(4) {
    override fun createChild(): KdTree<Vector3d, Vehicle> = VehicleTree()
    override fun getMin(data: Vehicle): Vector3d = data.treeBoundsMin
    override fun getMax(data: Vehicle): Vector3d = data.treeBoundsMax
    override fun getPoint(data: Vehicle): Vector3d = data.position
}