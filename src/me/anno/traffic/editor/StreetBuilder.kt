package me.anno.traffic.editor

import me.anno.maths.Maths
import me.anno.traffic.*
import me.anno.traffic.utils.SplineMaths.computeControlPoint
import me.anno.traffic.utils.SplineMaths.lerp3
import org.joml.Quaternionf
import org.joml.Vector3d
import kotlin.math.*

class StreetBuilder(val network: Network) {

    val position0 = Vector3d()
    val position1 = Vector3d()
    val position2 = Vector3d()

    val laneWidth = 4.0

    fun getPoint(i: Int, dxi: Int, n: Int, flip: Boolean): LanePoint {
        val jr = (dxi - (n - 1) * 0.5) * laneWidth
        return getPoint(i, jr, flip)
    }

    fun getPoint(t: Double, dx: Double, flip: Boolean): LanePoint {

        // calculate ideal position and rotation
        val p0 = lerp3(position0, position1, position2, max(t - 0.01, 0.0), Vector3d())
        val p1 = lerp3(position0, position1, position2, min(t + 0.01, 1.0), Vector3d())
        var angleY = p0.angleYTo(p1)

        // todo I can't believe these y-angles...
        //  -> debug the points along the lines...

        // todo why are the streets twisted sometimes???

        var angleX = lerp3(
            atan2(position1.y - position0.y, position1.distance(position0)),
            atan2(position2.y - position0.y, position2.distance(position0)),
            atan2(position2.y - position1.y, position2.distance(position1)), t
        )

        val position = lerp3(position0, position1, position2, t, Vector3d())
            .add(cos(angleY) * dx, 0.0, -sin(angleY) * dx)

        if (!flip) {
            angleY += Maths.PIf
            angleX = -angleX
        }

        val rotation = Quaternionf().rotateYXZ(angleY.toFloat(), angleX.toFloat(), 0f)
        return LanePoint(position, rotation, angleY, laneWidth * 0.5)
    }

    fun getPoint(i: Int, dx: Double, flip: Boolean): LanePoint {
        return getPoint(i * 0.5, dx, flip)
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

    fun createStreetInExpertMode(street: Street) =
        createStreetInExpertMode(0f, 1f, street)

    fun createStreetInExpertMode(t0: Float, t1: Float, street: Street): Street {

        if (t0 > 0f || t1 < 1f) {

            val p0 = Vector3d(position0)
            val p1 = Vector3d(position1)
            val p2 = Vector3d(position2)

            val fromPoint = getPoint(t0.toDouble(), 0.0, false)
            val toPoint = getPoint(t1.toDouble(), 0.0, false)
            val newControl =
                computeControlPoint(fromPoint.position, fromPoint.rotation, toPoint.position, toPoint.rotation)

            position0.set(fromPoint.position)
            position1.set(newControl)
            position2.set(toPoint.position)

            createStreetInExpertMode(0f, 1f, street)

            position0.set(p0)
            position1.set(p1)
            position2.set(p2)

            return street
        }

        val numLanes = 6
        val numReversed = 3
        val fromList = List(numLanes) { laneId ->
            getPoint(0, laneId, numLanes, flip = laneId < numReversed)
        }
        val controlList = List(numLanes) { laneId ->
            getPoint(1, laneId, numLanes, flip = laneId < numReversed)
        }
        val toList = List(numLanes) { laneId ->
            getPoint(2, laneId, numLanes, flip = laneId < numReversed)
        }

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
        val street = createStreetInPlannerMode()
        createStreetInExpertMode(street)
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