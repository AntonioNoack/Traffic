package me.anno.traffic.editor

import me.anno.maths.Maths
import me.anno.traffic.*
import me.anno.traffic.utils.SplineMaths.lerp3
import org.joml.Quaternionf
import org.joml.Vector3d
import kotlin.math.*

class StreetBuilder(val network: Network) {

    val position0 = Vector3d()
    val position1 = Vector3d()
    val position2 = Vector3d()

    val laneWidth = 4.0

    fun getPoint(f: Float, j: Int, n: Int, flip: Boolean): LanePoint {
        val jr = (j - (n - 1) * 0.5) * laneWidth
        return getPoint(f, jr, flip)
    }

    fun getPoint(f: Float, jr: Double, flip: Boolean): LanePoint {

        // calculate ideal position and rotation
        val p0 = lerp3(position0, position1, position2, max(f.toDouble() - 0.01, 0.0), Vector3d())
        val p1 = lerp3(position0, position1, position2, min(f.toDouble() + 0.01, 1.0), Vector3d())
        var angleY = p0.angleYTo(p1)

        var angleX = lerp3(
            atan2(position1.y - position0.y, position1.distance(position0)),
            atan2(position2.y - position0.y, position2.distance(position0)),
            atan2(position2.y - position1.y, position2.distance(position1)),
            f.toDouble()
        )

        val position = lerp3(position0, position1, position2, f.toDouble(), Vector3d())
            .add(cos(angleY) * jr, 0.0, -sin(angleY) * jr)

        if (!flip) {
            angleY += Maths.PIf
            angleX = -angleX
        }

        val rotation = Quaternionf().rotateYXZ(angleY.toFloat(), angleX.toFloat(), 0f)
        return LanePoint(position, rotation, angleY, laneWidth * 0.5)
    }

    fun getPoint(i: Int): StreetPoint {
        val base = when (i) {
            0 -> position0
            1 -> position1
            2 -> position2
            else -> throw IllegalStateException()
        }
        val point = network.getOrPutPoint(base, laneWidth * 0.5)
        base.set(point.position)
        return point
    }

    fun createStreetInExpertMode() =
        createStreetInExpertMode(0f, 1f)

    fun createStreetInExpertMode(t0: Float, t1: Float): Street {
        val numLanes = 6
        val numReversed = 3
        val fromList = List(numLanes) { laneId ->
            getPoint(t0, laneId, numLanes, flip = laneId < numReversed)
        }
        val controlList = List(numLanes) { laneId ->
            getPoint(0.5f, laneId, numLanes, flip = laneId < numReversed)
        }
        val toList = List(numLanes) { laneId ->
            getPoint(t1, laneId, numLanes, flip = laneId < numReversed)
        }

        val street = createStreetInPlannerMode()
        for (laneId in 0 until numLanes) {
            street.lanes += if (laneId < numReversed) {
                Lane(toList[laneId], controlList[laneId], fromList[laneId])
            } else {
                Lane(fromList[laneId], controlList[laneId], toList[laneId])
            }
        }
        return street
    }

    fun createStreetInPlannerMode(): Street {
        return Street(getPoint(0), getPoint(1), getPoint(2), ArrayList())
    }

    fun placeStreetInExpertMode(): Street {
        val street = createStreetInExpertMode()
        network.addStreet(street)
        return street
    }

    fun placeStreetInPlannerMode(): Street {
        val street = createStreetInPlannerMode()
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