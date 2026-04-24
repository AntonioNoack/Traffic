package me.anno.traffic.utils

import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f


fun Vector3f.f2(): String {
    return "(%.2f, %.2f, %.2f)".format(x, y, z)
}

fun Vector3d.f2(): String {
    return "(%.2f, %.2f, %.2f)".format(x, y, z)
}

fun Quaternionf.f2(): String {
    return "(%.2f, %.2f, %.2f, %.2f)".format(x, y, z, w)
}