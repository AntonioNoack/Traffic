package me.anno.traffic

import me.anno.maths.Maths.sq
import org.joml.Quaternionf
import org.joml.Vector3d

data class Lane(val from: LanePoint, val control: LanePoint, val to: LanePoint) {
    var currSection: CrossingSection? = null
    val vehicles = ArrayList<Vehicle>()

    fun mayEnterNextLane(nextLane: Lane): Boolean {
        val curr = currSection
        val next = nextLane.currSection
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
        return from.rotation
            .slerp(control.rotation, f1 / (f0 + f1), dst)
            .slerp(to.rotation, f2)
    }
}