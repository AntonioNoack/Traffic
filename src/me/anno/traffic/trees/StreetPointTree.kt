package me.anno.traffic.trees

import me.anno.graph.octtree.OctTree
import me.anno.traffic.StreetPoint
import org.joml.Vector3d

class StreetPointTree : OctTree<StreetPoint>(16) {
    override fun createChild() = StreetPointTree()
    override fun getMin(data: StreetPoint): Vector3d = data.position
    override fun getMax(data: StreetPoint): Vector3d = data.position
    override fun getPoint(data: StreetPoint): Vector3d = data.position
}