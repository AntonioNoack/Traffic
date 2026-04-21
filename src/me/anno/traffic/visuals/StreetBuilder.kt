package me.anno.traffic.visuals

import me.anno.maths.Maths.PIf
import me.anno.maths.Maths.mixAngle
import me.anno.traffic.Lane
import me.anno.traffic.LanePoint
import me.anno.traffic.Network
import me.anno.traffic.Street
import org.joml.Quaternionf
import org.joml.Vector3d
import kotlin.math.cos
import kotlin.math.sin

class StreetBuilder(val network: Network) {

    val position0 = Vector3d()
    val position1 = Vector3d()
    val position2 = Vector3d()

    val laneWidth = 4.0

    fun getPoint(i: Int, j: Int, n: Int, flip: Boolean): LanePoint {
        val jr = (j - (n - 1) * 0.5) * laneWidth

        // calculate ideal position
        val angle0 = position0.angleYTo(position1)
        val angle1 = position1.angleYTo(position2)
        var angle = when (i) {
            0 -> angle0
            1 -> mixAngle(angle0, angle1, 0.5)
            2 -> angle1
            else -> throw IllegalStateException()
        }

        val base = when (i) {
            0 -> position0
            1 -> position1
            2 -> position2
            else -> throw IllegalStateException()
        }

        val idealPosition = Vector3d(base)
            .add(cos(angle) * jr, 0.0, -sin(angle) * jr)
        val point0 = network.getPoint(idealPosition, laneWidth * 0.5)
        if (point0 != null) return point0

        if (flip) angle += PIf

        val point1 = LanePoint(idealPosition, Quaternionf().rotateY(angle.toFloat()), angle, laneWidth * 0.5)
        network.addPoint(point1)
        return point1
    }


    fun createStreet(): Street {
        val numLanes = 6
        val numReversed = 3
        val from = List(numLanes) { laneId ->
            getPoint(0, laneId, numLanes, flip = laneId < numReversed)
        }
        val control = List(numLanes) { laneId ->
            getPoint(1, laneId, numLanes, flip = laneId < numReversed)
        }
        val to = List(numLanes) { laneId ->
            getPoint(2, laneId, numLanes, flip = laneId < numReversed)
        }
        val lanes = List(6) { laneId ->
            if (laneId < numReversed) {
                Lane(to[laneId], control[laneId], from[laneId])
            } else {
                Lane(from[laneId], control[laneId], to[laneId])
            }
        }
        return Street(lanes)
    }

    fun placeStreet(): Street {
        val street = createStreet()
        network.addStreet(street)
        return street
    }

    fun extrudeCenter(d: Double) {
        val d = -0.5 * d
        val dx = position0.x + position2.x - 2.0 * position1.x
        val dy = position0.y + position2.y - 2.0 * position1.y
        val dz = position0.z + position2.z - 2.0 * position1.z
        position1.add(d * dx, d * dy, d * dz)
    }

}