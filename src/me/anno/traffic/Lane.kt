package me.anno.traffic

import me.anno.maths.Maths.sq
import me.anno.maths.optimization.GoldenSectionSearch
import org.joml.Quaternionf
import org.joml.Vector3d
import kotlin.math.abs

data class Lane(val from: LanePoint, val control: LanePoint, val to: LanePoint) {

    var crossingSection: CrossingSection? = null

    val approxLength: Double by lazy {
        val p0 = from.position
        val p1 = control.position
        val p2 = to.position
        val chord = p0.distance(p2)
        val net = p0.distance(p1) + p1.distance(p2)
        (chord + net) * 0.5
    }

    fun mayEnterNextLane(nextLane: Lane): Boolean {
        val curr = crossingSection
        val next = nextLane.crossingSection
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

    fun getClosestT(position: Vector3d, routeIndexF: Double): Double {
        val tmp = Vector3d()
        // Allow the search to continue slightly past the lane endpoint so the
        // caller can detect that the vehicle has crossed into the next lane.
        return GoldenSectionSearch.goldenSectionSearch(routeIndexF, 1.5, 1e-4, { t ->
            getPosition(t, 0.0, 0.0, tmp).distanceSquared(position)
        }, flipSign = false)
    }
}
