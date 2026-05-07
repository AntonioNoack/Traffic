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
        val dx = (dxi - (n - 1) * 0.5) * laneWidth

        // calculate ideal position and rotation
        val p0 = if (i < 2) position0 else position1
        val p1 = if (i < 1) position1 else position2
        var angleY = p0.angleYTo(p1)
        var angleX = atan2(p1.y - p0.y, p1.distance(p0))

        val position = when (i) {
            0 -> position0
            1 -> position1
            else -> position2
        }.add(cos(angleY) * dx, 0.0, -sin(angleY) * dx, Vector3d())

        if (!flip) {
            angleY += Maths.PIf
            angleX = -angleX
        }

        return createLanePoint(position, angleX, angleY)
    }

    fun getPoint(t: Double): LanePoint {

        // calculate ideal position and rotation
        val p0 = lerp3(position0, position1, position2, max(t - 0.01, 0.0), Vector3d())
        val p1 = lerp3(position0, position1, position2, min(t + 0.01, 1.0), Vector3d())
        val angleY = p1.angleYTo(p0)
        val angleX = atan2(p0.y - p1.y, p1.distance(p0))

        val position = lerp3(position0, position1, position2, t, Vector3d())
        return createLanePoint(position, angleX, angleY)
    }

    fun createLanePoint(position: Vector3d, angleX: Double, angleY: Double): LanePoint {
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

    fun createStreetInExpertMode(street: Street) =
        createStreetInExpertMode(0f, 1f, street)

    fun createStreetInExpertMode(t0: Float, t1: Float, street: Street): Street {

        if (t0 > 0f || t1 < 1f) {

            val p0 = Vector3d(position0)
            val p1 = Vector3d(position1)
            val p2 = Vector3d(position2)

            val fromPoint = getPoint(t0.toDouble())
            val toPoint = getPoint(t1.toDouble())
            val newControl = computeControlPoint(
                fromPoint.position, fromPoint.rotation,
                toPoint.position, toPoint.rotation
            )

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

    fun extrudeCenter(d: Double, dst: Vector3d = position1) {
        val d = -0.5 * d
        val dx = position0.x + position2.x - 2.0 * dst.x
        val dy = position0.y + position2.y - 2.0 * dst.y
        val dz = position0.z + position2.z - 2.0 * dst.z
        dst.add(d * dx, d * dy, d * dz)
    }


}