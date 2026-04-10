package me.anno.traffic.utils

import me.anno.graph.octtree.KdTree
import me.anno.graph.octtree.OctTree
import me.anno.traffic.LanePoint
import org.joml.Vector3d

class PointTree : OctTree<LanePoint>(16) {
    override fun createChild(): KdTree<Vector3d, LanePoint> = PointTree()
    override fun getMin(data: LanePoint): Vector3d = data.position
    override fun getMax(data: LanePoint): Vector3d = data.position
    override fun getPoint(data: LanePoint): Vector3d = data.position
}