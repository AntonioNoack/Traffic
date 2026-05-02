package me.anno.traffic

import me.anno.maths.Maths.mix
import me.anno.maths.Maths.pow
import me.anno.maths.optimization.GoldenSectionSearch
import me.anno.traffic.utils.SplineMaths.laneLength
import me.anno.traffic.utils.SplineMaths.lerp3
import me.anno.traffic.utils.SplineMaths.lerp3Diff
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import kotlin.math.min
import kotlin.math.sqrt

data class Lane(val from: LanePoint, val control: LanePoint, val to: LanePoint) {

    var supportedVehicleTypes = 0
    var crossingSection: CrossingSection? = null
    var maxSpeed = computeMaxSpeed()

    val approxLength = laneLength(from.position, control.position, to.position)

    fun mayEnterNextLane(nextLane: Lane?): Boolean {
        val curr = crossingSection
        val next = nextLane?.crossingSection
        if (curr == null || next == null) return true
        if (curr.crossing != next.crossing) return true
        return curr.crossing.mayDrive(curr.sectionId, next.sectionId)
    }

    fun getPosition(t: Double, dst: Vector3d): Vector3d {
        return lerp3(
            from.position,
            control.position,
            to.position,
            t, dst
        )
    }

    fun snapPositionToSurface(t: Double, dst: Vector3d): Vector3d {

        val pos0 = getPosition(t, Vector3d())
        val rot0 = getRotation(t.toFloat(), Quaternionf())

        val upDir = Vector3f(0f, 1f, 0f).rotate(rot0)
        val unitsBelowGround = upDir.dot(pos0) - upDir.dot(dst)
        dst.fma(unitsBelowGround, upDir)

        return dst
    }

    fun getDirection(t: Double): Vector3d {
        return lerp3Diff(from.position, control.position, to.position, t, Vector3d())
    }

    fun getDirection0(): Vector3f {
        val p0 = from.position
        val p1 = control.position
        return Vector3f(
            p1.x - p0.x,
            p1.y - p0.y,
            p1.z - p0.z
        )
    }

    fun getDirection1(): Vector3f {
        val p1 = control.position
        val p2 = to.position
        return Vector3f(
            p2.x - p1.x,
            p2.y - p1.y,
            p2.z - p1.z
        )
    }

    fun getPosition(t: Double, xf: Double, dst: Vector3d): Vector3d {
        getPosition(t, dst)
        val (x, y, z) = dst

        return dst.set(xf, 0.0, 0.0)
            .rotate(getRotation(t.toFloat(), Quaternionf()))
            .add(x, y, z)
    }

    fun getRotation(t: Float, dst: Quaternionf): Quaternionf {
        return lerp3(
            from.rotation,
            control.rotation,
            to.rotation,
            t, dst
        )
    }

    fun getClosestT(position: Vector3d, routeIndexF: Float): Float {
        return getClosestT(position, routeIndexF.toDouble()).toFloat()
    }

    fun getClosestT(position: Vector3d, routeIndexF: Double): Double {
        val tmp = Vector3d()
        // Allow the search to continue slightly past the lane endpoint so the
        // caller can detect that the vehicle has crossed into the next lane.
        return GoldenSectionSearch.goldenSectionSearch(routeIndexF, 1.5, 1e-4, { t ->
            getPosition(t, tmp).distanceSquared(position)
        }, flipSign = false)
    }

    private fun curvatureRadius(t: Double): Double {
        val p0 = from.position
        val p1 = control.position
        val p2 = to.position

        val d1 = Vector3d(
            mix(p1.x - p0.x, p2.x - p1.x, t),
            mix(p1.y - p0.y, p2.y - p1.y, t),
            mix(p1.z - p0.z, p2.z - p1.z, t)
        )

        val d2 = Vector3d(
            (p2.x - 2 * p1.x + p0.x),
            (p2.y - 2 * p1.y + p0.y),
            (p2.z - 2 * p1.z + p0.z)
        )

        val cross = d1.cross(d2, Vector3d())
        val num = cross.length() * 4.0
        val denom = pow(d1.length() * 2.0, 3.0)
        if (denom < 1e-6) return Double.POSITIVE_INFINITY

        val kappa = num / denom
        return if (kappa < 1e-6) Double.POSITIVE_INFINITY else 1.0 / kappa
    }

    private fun computeMaxSpeed(): Float {

        // 2 = cautious
        // 3 = normal
        // 4 = aggressive
        val accel = 3.0 // m/s² (comfortable cornering)

        val samples = listOf(0.25, 0.5, 0.75)
        val minRadius = samples.minOf { curvatureRadius(it) }

        val speed = sqrt(accel * minRadius)
        return min(speed.toFloat(), 130f / 3.6f)
    }

    fun getCurvature(): Float {
        val dir0 = getDirection0()
        val dir1 = getDirection1()
        return dir0.angleYTo(dir1) / approxLength
    }
}
