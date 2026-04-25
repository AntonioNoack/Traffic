package me.anno.traffic.visuals

import me.anno.engine.debug.DebugLine
import me.anno.engine.debug.DebugShapes
import me.anno.maths.Maths.posMod
import org.joml.Vector3d
import java.lang.Math.TAU
import kotlin.math.abs
import kotlin.math.min


fun debugDrawCircle(center: Vector3d, radius: Double) {
    // debug-draw circle
    val n = 30
    val pts = List(n) {
        Vector3d(radius, 0.0, 0.0)
            .rotateY(it * TAU / n)
            .add(center)
    }
    for (i in 0 until n) {
        val line = DebugLine(pts[i], pts[posMod(i + 1, n)], -1, 1e3f)
        DebugShapes.debugLines.add(line)
    }
}

fun absAngleDiff(angle: Double): Double {
    val v = posMod(abs(angle), TAU)
    return min(v, TAU - v)
}
