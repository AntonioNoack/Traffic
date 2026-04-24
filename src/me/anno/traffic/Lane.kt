package me.anno.traffic

import me.anno.maths.Maths.mix
import me.anno.maths.Maths.pow
import me.anno.maths.Maths.sq
import me.anno.maths.optimization.GoldenSectionSearch
import org.joml.Quaternionf
import org.joml.Vector3d
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sqrt

data class Lane(val from: LanePoint, val control: LanePoint, val to: LanePoint) {

    var crossingSection: CrossingSection? = null
    var maxSpeed = computeMaxSpeed()

    val approxLength: Float by lazy {
        val p0 = from.position
        val p1 = control.position
        val p2 = to.position
        val chord = p0.distance(p2)
        val net = p0.distance(p1) + p1.distance(p2)
        (chord + net).toFloat() * 0.5f
    }

    fun mayEnterNextLane(nextLane: Lane?): Boolean {
        val curr = crossingSection
        val next = nextLane?.crossingSection
        if (curr == null || next == null) return true
        if (curr.crossing != next.crossing) return true
        return curr.crossing.mayDrive(curr.sectionId, next.sectionId)
    }

    fun getPosition(t: Double, x: Double, y: Double, dst: Vector3d): Vector3d {
        val f0 = sq(1f - t)
        val f1 = 2f * (1f - t) * t
        val f2 = t * t

        dst.set(x, y, 0.0)
            .rotate(getRotation(t.toFloat(), Quaternionf()))

        return dst
            .fma(f0, from.position)
            .fma(f1, control.position)
            .fma(f2, to.position)
    }

    fun getRotation(t: Float, dst: Quaternionf): Quaternionf {
        val f0 = sq(1f - t)
        val f1 = 2f * (1f - t) * t
        val f2 = t * t
        val f01 = f0 + f1
        if (abs(f01) < 1e-30f) {
            return dst.set(to.rotation)
        }
        return from.rotation
            .slerp(control.rotation, f1 / f01, dst)
            .slerp(to.rotation, f2)
    }

    fun getClosestT(position: Vector3d, routeIndexF: Float): Float {
        return getClosestT(position, routeIndexF.toDouble()).toFloat()
    }

    fun getClosestT(position: Vector3d, routeIndexF: Double): Double {
        val tmp = Vector3d()
        // Allow the search to continue slightly past the lane endpoint so the
        // caller can detect that the vehicle has crossed into the next lane.
        return GoldenSectionSearch.goldenSectionSearch(routeIndexF, 1.5, 1e-4, { t ->
            getPosition(t, 0.0, 0.0, tmp).distanceSquared(position)
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
}
