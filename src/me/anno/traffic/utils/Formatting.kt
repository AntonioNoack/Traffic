package me.anno.traffic.utils

import me.anno.utils.types.Floats.f2s
import org.joml.Quaternionf
import org.joml.Vector
import org.joml.Vector2d

val Quaternionf.ry get() = getEulerAngleYXZvY()

val Vector.xz: Vector2d
    get() {
        check(numComponents >= 3)
        return Vector2d(getComp(0), getComp(2))
    }

fun Vector.f2(): String {
    val result = StringBuilder()
    result.append('(')
    for (i in 0 until numComponents) {
        if (i > 0) result.append(", ")
        result.append(getComp(i).f2s())
    }
    result.append(')')
    return result.toString()
}
