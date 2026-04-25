package me.anno.traffic.trees

import me.anno.graph.octtree.KdTree
import me.anno.graph.octtree.OctTree
import me.anno.traffic.Vehicle
import org.joml.Vector3d

class CollisionVehicleTree : OctTree<Vehicle>(4) {
    override fun createChild(): KdTree<Vector3d, Vehicle> = CollisionVehicleTree()
    override fun getMin(data: Vehicle): Vector3d = data.collisionBoundsMin
    override fun getMax(data: Vehicle): Vector3d = data.collisionBoundsMax
    override fun getPoint(data: Vehicle): Vector3d = data.position
}