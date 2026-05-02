package me.anno.traffic.utils

import me.anno.maths.Maths
import org.joml.Quaternionf
import org.joml.Vector3d
import kotlin.math.abs

object SplineMaths {

    fun laneLength(p0: Vector3d, p1: Vector3d, p2: Vector3d): Float {
        val chord = p0.distance(p2)
        val net = p0.distance(p1) + p1.distance(p2)
        return (chord + net).toFloat() * 0.5f
    }

    fun lerp3(p0: Double, p1: Double, p2: Double, t: Double): Double {
        val f0 = Maths.sq(1f - t)
        val f1 = 2f * (1f - t) * t
        val f2 = t * t

        return f0 * p0 + f1 * p1 + f2 * p2
    }

    fun lerp3(p0: Vector3d, p1: Vector3d, p2: Vector3d, t: Double, dst: Vector3d): Vector3d {
        val f0 = Maths.sq(1f - t)
        val f1 = 2f * (1f - t) * t
        val f2 = t * t

        return dst.set(0.0)
            .fma(f0, p0)
            .fma(f1, p1)
            .fma(f2, p2)
    }

    fun lerp3(p0: Quaternionf, p1: Quaternionf, p2: Quaternionf, t: Float, dst: Quaternionf): Quaternionf {
        val f0 = Maths.sq(1f - t)
        val f1 = 2f * (1f - t) * t
        val f2 = t * t
        val f01 = f0 + f1
        if (abs(f01) < 1e-30f) {
            return dst.set(p2)
        }
        return p0
            .slerp(p1, f1 / f01, dst)
            .slerp(p2, f2)
    }

    fun lerp3Diff(p0: Vector3d, p1: Vector3d, p2: Vector3d, t: Double, dst: Vector3d): Vector3d {
        return dst.set(
            Maths.mix(p1.x - p0.x, p2.x - p1.x, t),
            Maths.mix(p1.y - p0.y, p2.y - p1.y, t),
            Maths.mix(p1.z - p0.z, p2.z - p1.z, t),
        )
    }

    fun computeControlPoint(
        pos0: Vector3d, rot0: Quaternionf,
        pos2: Vector3d, rot2: Quaternionf
    ): Vector3d {

        val d0 = Vector3d(0f, 0f, 1f).rotate(rot0).normalize()
        val d2 = Vector3d(0f, 0f, 1f).rotate(rot2).normalize()

        val a00 = d0.dot(d0)
        val a01 = d0.dot(d2)
        val a11 = d2.dot(d2)

        val b0 = 2.0 * (d0.dot(pos2) - d0.dot(pos0))
        val b1 = 2.0 * (d2.dot(pos2) - d2.dot(pos0))

        val det = a00 * a11 - a01 * a01

        // parallel case / degenerate
        if (abs(det) < 1e-8) {
            val dir = Vector3d(d0).normalize()

            val delta = Vector3d(pos2).sub(pos0)
            val lengthAlong = delta.dot(dir)

            // Place control point halfway along the line
            return Vector3d(pos0).fma(0.5 * lengthAlong, dir)
        }

        // curve
        val lambda0 = (b0 * a11 - b1 * a01) / det
        val lambda2 = (-b0 * a01 + b1 * a00) / det

        val p1a = Vector3d(pos0).fma(0.5 * lambda0, d0)
        val p1b = Vector3d(pos2).fma(-0.5 * lambda2, d2)

        // Average improves numerical stability
        return p1a.add(p1b).mul(0.5)
    }
}